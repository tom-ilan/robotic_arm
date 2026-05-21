"""
Robot Arm Controller UI
-----------------------
Pygame-based UI for setting waypoints and running a robotic arm sequence.

Layout:
  LEFT  - Timeline panel (ordered list of waypoints, editable)
  RIGHT - XY view (top) + XZ view (bottom) with plotted points

Controls:
  - Click the XY or XZ views to place / move the projected position
  - ADD POINT  : appends a new waypoint at the current cursor crosshair position
  - DELETE      : removes the selected point
  - RUN         : executes the sequence on the real arm (or simulates if no serial)
  - Click any timeline row to select that point for editing / deletion

Serial port is configured at the top of this file.
"""

import pygame
import sys
import math
import time
import threading

# ── Serial / arm integration ──────────────────────────────────────────────────
# Set USE_REAL_ARM = True and configure the port to drive hardware.
USE_REAL_ARM = True
SERIAL_PORT  = '/dev/cu.usbmodem1101'
RUN_PAUSE_SECONDS = 1.2

# Import real modules only when needed so the UI works standalone too
if USE_REAL_ARM:
    import serial
    import go_to

# ── Workspace bounds (mm) ─────────────────────────────────────────────────────
X_MIN, X_MAX = -300, 300
Y_MIN, Y_MAX = -300, 300
Z_MIN, Z_MAX =    0, 400

# ── Colors ────────────────────────────────────────────────────────────────────
BG          = ( 18,  20,  28)
PANEL_BG    = ( 26,  29,  42)
BORDER      = ( 55,  60,  90)
GRID        = ( 38,  42,  62)
ACCENT      = ( 80, 180, 255)
ACCENT2     = (255, 140,  60)
WHITE       = (220, 225, 240)
GREY        = (110, 115, 140)
RED         = (255,  80,  80)
GREEN       = ( 60, 220, 120)
SEL_BG      = ( 40,  60, 100)
PATH_COL    = ( 80, 180, 255)
POINT_COL   = (255, 210,  60)
POINT_SEL   = (255, 140,  60)
CURSOR_COL  = ( 55, 200, 130)
LABEL_COL   = (160, 165, 200)
PLAYHEAD    = (255,  80,  80)

# ── Layout constants ──────────────────────────────────────────────────────────
TIMELINE_W  = 270
PADDING     = 12
ROW_H       = 42
HEADER_H    = 40
BUTTON_H    = 36
FONT_SM     = 13
FONT_MD     = 15
FONT_LG     = 18

# ── Data ──────────────────────────────────────────────────────────────────────
class Waypoint:
    _counter = 1
    def __init__(self, x=0, y=0, z=100):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.label = f"P{Waypoint._counter:02d}"
        Waypoint._counter += 1

    def as_ints(self):
        return int(round(self.x)), int(round(self.y)), int(round(self.z))


# ── App ───────────────────────────────────────────────────────────────────────
class RobotArmUI:
    def __init__(self):
        pygame.init()
        self.W, self.H = 1100, 700
        self.screen = pygame.display.set_mode((self.W, self.H), pygame.RESIZABLE)
        pygame.display.set_caption("Robot Arm Controller")

        self.font_sm = pygame.font.SysFont("SF Pro Display, Helvetica, Arial", FONT_SM)
        self.font_md = pygame.font.SysFont("SF Pro Display, Helvetica, Arial", FONT_MD)
        self.font_lg = pygame.font.SysFont("SF Pro Display, Helvetica, Arial", FONT_LG, bold=True)
        self.font_mono = pygame.font.SysFont("Menlo, Consolas, Courier New", FONT_SM)

        self.waypoints   = []       # list of Waypoint
        self.selected    = None     # index of selected waypoint
        self.run_index   = None     # which point the arm is currently moving to
        self.running     = False
        self.run_thread  = None
        self.status_msg  = "Ready"
        self.status_ok   = True

        # Cross-hair position in world space (used when placing new points)
        self.cursor_x = 0.0
        self.cursor_y = 0.0
        self.cursor_z = 100.0

        # Drag state
        self.dragging_point = None  # (index, view)  view in ('xy','xz')
        self.active_view    = None  # 'xy' or 'xz'

        # Timeline scroll
        self.tl_scroll = 0

        # Edit box state  (None or {'idx':int, 'field':'x'/'y'/'z', 'buf':str})
        self.edit = None

        self.clock = pygame.time.Clock()

    # ── Geometry helpers ──────────────────────────────────────────────────────

    def _view_rects(self):
        """Return (xy_rect, xz_rect) for the two visualisation panels."""
        W, H = self.screen.get_size()
        rx = TIMELINE_W + PADDING
        ry = PADDING + HEADER_H
        rw = W - TIMELINE_W - PADDING * 2
        rh = (H - PADDING * 2 - HEADER_H - PADDING) // 2
        xy = pygame.Rect(rx, ry, rw, rh)
        xz = pygame.Rect(rx, ry + rh + PADDING, rw, H - ry - rh - PADDING * 2)
        return xy, xz

    def _world_to_xy(self, wx, wy, rect):
        px = rect.x + (wx - X_MIN) / (X_MAX - X_MIN) * rect.w
        py = rect.y + (1 - (wy - Y_MIN) / (Y_MAX - Y_MIN)) * rect.h
        return int(px), int(py)

    def _world_to_xz(self, wx, wz, rect):
        px = rect.x + (wx - X_MIN) / (X_MAX - X_MIN) * rect.w
        pz = rect.y + (1 - (wz - Z_MIN) / (Z_MAX - Z_MIN)) * rect.h
        return int(px), int(pz)

    def _xy_to_world(self, px, py, rect):
        wx = X_MIN + (px - rect.x) / rect.w * (X_MAX - X_MIN)
        wy = Y_MIN + (1 - (py - rect.y) / rect.h) * (Y_MAX - Y_MIN)
        return wx, wy

    def _xz_to_world(self, px, pz, rect):
        wx = X_MIN + (px - rect.x) / rect.w * (X_MAX - X_MIN)
        wz = Z_MIN + (1 - (pz - rect.y) / rect.h) * (Z_MAX - Z_MIN)
        return wx, wz

    def _clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    # ── Drawing ───────────────────────────────────────────────────────────────

    def _draw_grid(self, rect, label_x, label_y, x_range, y_range, steps=5):
        """Draw axis grid lines and labels inside rect."""
        pygame.draw.rect(self.screen, PANEL_BG, rect)
        pygame.draw.rect(self.screen, BORDER, rect, 1)

        for i in range(steps + 1):
            t = i / steps
            # vertical
            gx = rect.x + int(t * rect.w)
            pygame.draw.line(self.screen, GRID, (gx, rect.y), (gx, rect.y + rect.h))
            val = x_range[0] + t * (x_range[1] - x_range[0])
            lbl = self.font_sm.render(f"{val:.0f}", True, GREY)
            self.screen.blit(lbl, (gx - lbl.get_width() // 2, rect.bottom + 2))
            # horizontal
            gy = rect.y + int(t * rect.h)
            pygame.draw.line(self.screen, GRID, (rect.x, gy), (rect.x + rect.w, gy))
            val2 = y_range[1] - t * (y_range[1] - y_range[0])
            lbl2 = self.font_sm.render(f"{val2:.0f}", True, GREY)
            self.screen.blit(lbl2, (rect.x - lbl2.get_width() - 4, gy - 6))

        # axis origin cross
        ox, oy = self._world_to_xy(0, 0, rect) if label_y == 'Y' else self._world_to_xz(0, 0, rect)
        pygame.draw.line(self.screen, BORDER, (ox, rect.y), (ox, rect.bottom), 1)
        pygame.draw.line(self.screen, BORDER, (rect.x, oy), (rect.right, oy), 1)

        # labels
        xl = self.font_md.render(label_x, True, LABEL_COL)
        yl = self.font_md.render(label_y, True, LABEL_COL)
        self.screen.blit(xl, (rect.right - xl.get_width() - 6, rect.bottom - xl.get_height() - 4))
        self.screen.blit(yl, (rect.x + 4, rect.y + 4))

    def _draw_path(self, wps, view, rect):
        if len(wps) < 2:
            return
        pts = []
        for wp in wps:
            if view == 'xy':
                pts.append(self._world_to_xy(wp.x, wp.y, rect))
            else:
                pts.append(self._world_to_xz(wp.x, wp.z, rect))
        for i in range(len(pts) - 1):
            pygame.draw.line(self.screen, PATH_COL, pts[i], pts[i+1], 2)

    def _draw_points(self, wps, view, rect):
        for i, wp in enumerate(wps):
            if view == 'xy':
                p = self._world_to_xy(wp.x, wp.y, rect)
            else:
                p = self._world_to_xz(wp.x, wp.z, rect)

            col = POINT_SEL if i == self.selected else POINT_COL
            if i == self.run_index and self.running:
                col = GREEN
            pygame.draw.circle(self.screen, col, p, 8)
            pygame.draw.circle(self.screen, WHITE, p, 8, 2)
            idx_lbl = self.font_sm.render(wp.label, True, BG)
            self.screen.blit(idx_lbl, (p[0] - idx_lbl.get_width()//2, p[1] - idx_lbl.get_height()//2))

    def _draw_cursor(self, view, rect):
        if view == 'xy':
            p = self._world_to_xy(self.cursor_x, self.cursor_y, rect)
        else:
            p = self._world_to_xz(self.cursor_x, self.cursor_z, rect)
        pygame.draw.line(self.screen, CURSOR_COL, (p[0], rect.y), (p[0], rect.bottom), 1)
        pygame.draw.line(self.screen, CURSOR_COL, (rect.x, p[1]), (rect.right, p[1]), 1)
        pygame.draw.circle(self.screen, CURSOR_COL, p, 5, 2)

    def _draw_timeline(self, rect):
        pygame.draw.rect(self.screen, PANEL_BG, rect)
        pygame.draw.rect(self.screen, BORDER, rect, 1)

        title = self.font_lg.render("WAYPOINTS", True, WHITE)
        self.screen.blit(title, (rect.x + PADDING, rect.y + 10))

        # Buttons row
        btn_y = rect.y + HEADER_H + 4
        self._btn_add    = self._draw_button("+ ADD",    rect.x + PADDING,             btn_y, 80, BUTTON_H, GREEN)
        self._btn_delete = self._draw_button("DEL",      rect.x + PADDING + 86,        btn_y, 52, BUTTON_H, RED)
        self._btn_clear  = self._draw_button("CLR",      rect.x + PADDING + 86 + 58,   btn_y, 52, BUTTON_H, (140, 60, 60))
        self._btn_run    = self._draw_button("▶  RUN" if not self.running else "■ STOP",
                                             rect.x + PADDING, btn_y + BUTTON_H + 6, rect.w - PADDING * 2, BUTTON_H,
                                             ACCENT if not self.running else RED)

        # Timeline rows
        list_y = btn_y + BUTTON_H * 2 + 14
        clip = pygame.Rect(rect.x, list_y, rect.w, rect.bottom - list_y - PADDING)
        self.screen.set_clip(clip)

        for i, wp in enumerate(self.waypoints):
            ry = list_y + i * ROW_H - self.tl_scroll
            if ry + ROW_H < list_y or ry > rect.bottom:
                continue
            row_rect = pygame.Rect(rect.x + 4, ry, rect.w - 8, ROW_H - 2)

            if i == self.selected:
                pygame.draw.rect(self.screen, SEL_BG, row_rect, border_radius=4)
            if i == self.run_index and self.running:
                pygame.draw.rect(self.screen, (30, 80, 50), row_rect, border_radius=4)
            pygame.draw.rect(self.screen, BORDER, row_rect, 1, border_radius=4)

            # Dot indicator
            dot_col = POINT_SEL if i == self.selected else POINT_COL
            if i == self.run_index and self.running:
                dot_col = GREEN
            pygame.draw.circle(self.screen, dot_col,
                                (row_rect.x + 14, row_rect.centery), 5)

            lbl = self.font_md.render(wp.label, True, WHITE)
            self.screen.blit(lbl, (row_rect.x + 26, ry + 5))

            coords = self.font_mono.render(
                f"X{wp.x:+07.1f}  Y{wp.y:+07.1f}  Z{wp.z:+07.1f}", True, GREY)
            self.screen.blit(coords, (row_rect.x + 26, ry + 22))

            # Playhead arrow
            if i == self.run_index and self.running:
                pygame.draw.polygon(self.screen, PLAYHEAD, [
                    (row_rect.right - 14, row_rect.centery),
                    (row_rect.right - 22, row_rect.centery - 5),
                    (row_rect.right - 22, row_rect.centery + 5),
                ])

        self.screen.set_clip(None)

        # Inline coord editor
        if self.edit is not None and self.selected is not None:
            self._draw_edit_box(rect)

        # Status bar
        status_col = GREEN if self.status_ok else RED
        st = self.font_sm.render(self.status_msg, True, status_col)
        self.screen.blit(st, (rect.x + PADDING, rect.bottom - 20))

    def _draw_button(self, text, x, y, w, h, color):
        r = pygame.Rect(x, y, w, h)
        hover = r.collidepoint(pygame.mouse.get_pos())
        c = tuple(min(255, v + 30) for v in color) if hover else color
        pygame.draw.rect(self.screen, c, r, border_radius=5)
        pygame.draw.rect(self.screen, WHITE, r, 1, border_radius=5)
        lbl = self.font_md.render(text, True, BG if hover else WHITE)
        self.screen.blit(lbl, (r.centerx - lbl.get_width()//2, r.centery - lbl.get_height()//2))
        return r

    def _draw_edit_box(self, tl_rect):
        """Small floating editor for XYZ of selected point."""
        wp = self.waypoints[self.selected]
        ex = tl_rect.x + PADDING
        ey = tl_rect.y + HEADER_H + BUTTON_H * 2 + 22 + self.selected * ROW_H - self.tl_scroll
        ey = max(tl_rect.y + HEADER_H + BUTTON_H * 2 + 20, ey)
        box = pygame.Rect(ex - 2, ey - 2, tl_rect.w - PADDING - 2, 80)
        pygame.draw.rect(self.screen, (22, 30, 50), box, border_radius=6)
        pygame.draw.rect(self.screen, ACCENT, box, 1, border_radius=6)
        for fi, (field, val) in enumerate([('x', wp.x), ('y', wp.y), ('z', wp.z)]):
            fx = ex + fi * 82
            fy = ey + 6
            active = self.edit['field'] == field
            frect = pygame.Rect(fx, fy, 76, 26)
            pygame.draw.rect(self.screen, SEL_BG if active else PANEL_BG, frect, border_radius=3)
            pygame.draw.rect(self.screen, ACCENT if active else BORDER, frect, 1, border_radius=3)
            buf = self.edit['buf'] if active else f"{val:.1f}"
            cursor_str = buf + ("|" if active and (pygame.time.get_ticks() // 500) % 2 == 0 else "")
            t = self.font_mono.render(field.upper() + ":" + cursor_str, True, WHITE if active else GREY)
            self.screen.blit(t, (fx + 4, fy + 6))
        hint = self.font_sm.render("Tab/click field  Enter=confirm  Esc=cancel", True, GREY)
        self.screen.blit(hint, (ex, ey + 38))

    def _draw_selected_info(self):
        """HUD showing selected point's live coords."""
        if self.selected is None or self.selected >= len(self.waypoints):
            return
        wp = self.waypoints[self.selected]
        W, _ = self.screen.get_size()
        info = f"{wp.label}  X:{wp.x:+.1f}  Y:{wp.y:+.1f}  Z:{wp.z:+.1f}"
        t = self.font_md.render(info, True, ACCENT)
        self.screen.blit(t, (TIMELINE_W + PADDING + 8, 12))

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self):
        while True:
            self.clock.tick(60)
            for event in pygame.event.get():
                self._handle_event(event)

            W, H = self.screen.get_size()
            self.screen.fill(BG)

            tl_rect = pygame.Rect(0, 0, TIMELINE_W, H)
            xy_rect, xz_rect = self._view_rects()

            # Draw views
            self._draw_grid(xy_rect, 'X (mm)', 'Y (mm)', (X_MIN, X_MAX), (Y_MIN, Y_MAX))
            self._draw_grid(xz_rect, 'X (mm)', 'Z (mm)', (X_MIN, X_MAX), (Z_MIN, Z_MAX))

            # View labels
            for rect, label in [(xy_rect, "TOP VIEW  (XY)"), (xz_rect, "FRONT VIEW  (XZ)")]:
                lt = self.font_lg.render(label, True, LABEL_COL)
                self.screen.blit(lt, (rect.x + 8, rect.y + 6))

            self._draw_path(self.waypoints, 'xy', xy_rect)
            self._draw_path(self.waypoints, 'xz', xz_rect)
            self._draw_points(self.waypoints, 'xy', xy_rect)
            self._draw_points(self.waypoints, 'xz', xz_rect)
            self._draw_cursor('xy', xy_rect)
            self._draw_cursor('xz', xz_rect)

            self._draw_timeline(tl_rect)
            self._draw_selected_info()

            pygame.display.flip()

    # ── Events ────────────────────────────────────────────────────────────────

    def _handle_event(self, event):
        xy_rect, xz_rect = self._view_rects()
        W, H = self.screen.get_size()
        tl_rect = pygame.Rect(0, 0, TIMELINE_W, H)

        if event.type == pygame.QUIT:
            pygame.quit(); sys.exit()

        elif event.type == pygame.KEYDOWN:
            self._handle_key(event)

        elif event.type == pygame.MOUSEBUTTONDOWN:
            mx, my = event.pos
            if event.button == 1:
                # Buttons
                if hasattr(self, '_btn_add') and self._btn_add.collidepoint(event.pos):
                    self._add_waypoint()
                elif hasattr(self, '_btn_delete') and self._btn_delete.collidepoint(event.pos):
                    self._delete_selected()
                elif hasattr(self, '_btn_clear') and self._btn_clear.collidepoint(event.pos):
                    self.waypoints.clear(); self.selected = None; self.tl_scroll = 0
                elif hasattr(self, '_btn_run') and self._btn_run.collidepoint(event.pos):
                    if self.running:
                        self._stop_run()
                    else:
                        self._start_run()
                elif xy_rect.collidepoint(mx, my):
                    self._click_view(mx, my, xy_rect, 'xy')
                elif xz_rect.collidepoint(mx, my):
                    self._click_view(mx, my, xz_rect, 'xz')
                elif tl_rect.collidepoint(mx, my):
                    self._click_timeline(mx, my, tl_rect)

            elif event.button == 4:  # scroll up
                if tl_rect.collidepoint(event.pos):
                    self.tl_scroll = max(0, self.tl_scroll - ROW_H)
            elif event.button == 5:  # scroll down
                if tl_rect.collidepoint(event.pos):
                    max_scroll = max(0, len(self.waypoints) * ROW_H - (H // 2))
                    self.tl_scroll = min(max_scroll, self.tl_scroll + ROW_H)

        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                self.dragging_point = None
                self.active_view = None

        elif event.type == pygame.MOUSEMOTION:
            mx, my = event.pos
            # Update crosshair
            if xy_rect.collidepoint(mx, my):
                self.cursor_x, self.cursor_y = self._xy_to_world(mx, my, xy_rect)
                self.cursor_x = self._clamp(self.cursor_x, X_MIN, X_MAX)
                self.cursor_y = self._clamp(self.cursor_y, Y_MIN, Y_MAX)
            elif xz_rect.collidepoint(mx, my):
                self.cursor_x, self.cursor_z = self._xz_to_world(mx, my, xz_rect)
                self.cursor_x = self._clamp(self.cursor_x, X_MIN, X_MAX)
                self.cursor_z = self._clamp(self.cursor_z, Z_MIN, Z_MAX)

            # Drag point
            if self.dragging_point is not None:
                idx, view = self.dragging_point
                wp = self.waypoints[idx]
                if view == 'xy' and xy_rect.collidepoint(mx, my):
                    wp.x, wp.y = self._xy_to_world(mx, my, xy_rect)
                    wp.x = self._clamp(wp.x, X_MIN, X_MAX)
                    wp.y = self._clamp(wp.y, Y_MIN, Y_MAX)
                elif view == 'xz' and xz_rect.collidepoint(mx, my):
                    wp.x, wp.z = self._xz_to_world(mx, my, xz_rect)
                    wp.x = self._clamp(wp.x, X_MIN, X_MAX)
                    wp.z = self._clamp(wp.z, Z_MIN, Z_MAX)

    def _click_view(self, mx, my, rect, view):
        """Click in a visualisation panel: pick existing point or start fresh."""
        # Check if near an existing point
        for i, wp in enumerate(self.waypoints):
            if view == 'xy':
                p = self._world_to_xy(wp.x, wp.y, rect)
            else:
                p = self._world_to_xz(wp.x, wp.z, rect)
            if math.hypot(mx - p[0], my - p[1]) < 12:
                self.selected = i
                self.dragging_point = (i, view)
                self.edit = None
                return
        # Else: update cursor only (use Add button to commit)
        if view == 'xy':
            cx, cy = self._xy_to_world(mx, my, rect)
            self.cursor_x = self._clamp(cx, X_MIN, X_MAX)
            self.cursor_y = self._clamp(cy, Y_MIN, Y_MAX)
        else:
            cx, cz = self._xz_to_world(mx, my, rect)
            self.cursor_x = self._clamp(cx, X_MIN, X_MAX)
            self.cursor_z = self._clamp(cz, Z_MIN, Z_MAX)
        self.selected = None
        self.edit = None

    def _click_timeline(self, mx, my, tl_rect):
        btn_y = tl_rect.y + HEADER_H + 4
        list_y = btn_y + BUTTON_H * 2 + 14
        for i in range(len(self.waypoints)):
            ry = list_y + i * ROW_H - self.tl_scroll
            row_rect = pygame.Rect(tl_rect.x + 4, ry, tl_rect.w - 8, ROW_H - 2)
            if row_rect.collidepoint(mx, my):
                if self.selected == i:
                    # Open inline editor
                    self.edit = {'idx': i, 'field': 'x', 'buf': f"{self.waypoints[i].x:.1f}"}
                else:
                    self.selected = i
                    self.edit = None
                return
        # Check field clicks inside edit box
        if self.edit and self.selected is not None:
            wp = self.waypoints[self.selected]
            ey = list_y + self.selected * ROW_H - self.tl_scroll
            ex = tl_rect.x + PADDING
            for fi, field in enumerate(['x', 'y', 'z']):
                frect = pygame.Rect(ex + fi * 82, ey + 6, 76, 26)
                if frect.collidepoint(mx, my):
                    self._commit_edit()
                    val = getattr(wp, field)
                    self.edit = {'idx': self.selected, 'field': field, 'buf': f"{val:.1f}"}

    def _handle_key(self, event):
        if self.edit is not None:
            # Text input for coord editing
            if event.key == pygame.K_RETURN:
                self._commit_edit()
            elif event.key == pygame.K_ESCAPE:
                self.edit = None
            elif event.key == pygame.K_TAB:
                self._commit_edit()
                fields = ['x', 'y', 'z']
                fi = fields.index(self.edit['field']) if self.edit else 0
                nf = fields[(fi + 1) % 3]
                wp = self.waypoints[self.selected]
                self.edit = {'idx': self.selected, 'field': nf, 'buf': f"{getattr(wp, nf):.1f}"}
            elif event.key == pygame.K_BACKSPACE:
                self.edit['buf'] = self.edit['buf'][:-1]
            else:
                ch = event.unicode
                if ch in '0123456789.-':
                    self.edit['buf'] += ch
        else:
            if event.key == pygame.K_DELETE or event.key == pygame.K_BACKSPACE:
                self._delete_selected()
            elif event.key == pygame.K_a:
                self._add_waypoint()
            elif event.key == pygame.K_RETURN:
                self._start_run()
            elif event.key == pygame.K_ESCAPE:
                self._stop_run()
            elif event.key == pygame.K_UP and self.selected is not None:
                self.selected = max(0, self.selected - 1)
            elif event.key == pygame.K_DOWN and self.selected is not None:
                self.selected = min(len(self.waypoints) - 1, self.selected + 1)

    def _commit_edit(self):
        if self.edit is None:
            return
        try:
            val = float(self.edit['buf'])
            wp = self.waypoints[self.edit['idx']]
            field = self.edit['field']
            if field == 'x':
                wp.x = self._clamp(val, X_MIN, X_MAX)
            elif field == 'y':
                wp.y = self._clamp(val, Y_MIN, Y_MAX)
            elif field == 'z':
                wp.z = self._clamp(val, Z_MIN, Z_MAX)
        except ValueError:
            pass
        self.edit = None

    # ── Waypoint management ───────────────────────────────────────────────────

    def _add_waypoint(self):
        wp = Waypoint(self.cursor_x, self.cursor_y, self.cursor_z)
        self.waypoints.append(wp)
        self.selected = len(self.waypoints) - 1
        self.status_msg = f"Added {wp.label}"

    def _delete_selected(self):
        if self.selected is not None and self.waypoints:
            removed = self.waypoints.pop(self.selected)
            self.selected = min(self.selected, len(self.waypoints) - 1) if self.waypoints else None
            self.edit = None
            self.status_msg = f"Deleted {removed.label}"

    # ── Run / stop ────────────────────────────────────────────────────────────

    def _start_run(self):
        if not self.waypoints:
            self.status_msg = "No waypoints!"; self.status_ok = False; return
        if self.running:
            return
        self.running = True
        self.status_ok = True
        self.run_thread = threading.Thread(target=self._run_sequence, daemon=True)
        self.run_thread.start()

    def _stop_run(self):
        self.running = False
        self.run_index = None
        self.status_msg = "Stopped"

    def _run_sequence(self):
        if USE_REAL_ARM:
            with serial.Serial(SERIAL_PORT) as ser:
                for i, wp in enumerate(self.waypoints):
                    if not self.running:
                        break
                    self.run_index = i
                    self.status_msg = f"Moving to {wp.label}…"
                    x, y, z = wp.as_ints()
                    go_to.go_to_precise(ser, x_mm=x, y_mm=y, z_mm=z)
                    # pause to give the arm time to reach the waypoint before issuing the next command
                    time.sleep(RUN_PAUSE_SECONDS)
        else:
            # Simulation: pause 1.2 s per waypoint
            for i, wp in enumerate(self.waypoints):
                if not self.running:
                    break
                self.run_index = i
                self.status_msg = f"[SIM] Moving to {wp.label} ({wp.x:.0f}, {wp.y:.0f}, {wp.z:.0f})"
                time.sleep(1.2)

        self.running = False
        self.run_index = None
        self.status_msg = "Sequence complete ✓" if self.waypoints else "Ready"


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    RobotArmUI().run()