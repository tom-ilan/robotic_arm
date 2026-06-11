"""
🦾 Robotic Arm Controller & Visualizer (ArControllerGUI)
------------------------------------------------------
An interactive, high-performance Pygame dashboard for controlling a 3-axis arm.
Features dual-plane XY and XZ projections, real-time geometric IK solvers,
safety-bound validation (Reach & Rotational limits), and a resilient serial sequencer.

Created as part of the implementation plan for robust arm control.
"""

import pygame
import sys
import math
import time
import threading
import kinematics
import go_to
import serial

# ── Global Configurations & Physical Limits ───────────────────────────────────
USE_REAL_ARM = True
SERIAL_PORT  = '/dev/cu.usbmodem1101'
BAUD_RATE    = 115200  # High speed for smooth real-time control

# Arm Dimensions (mm) - Imported directly from kinematics.py for single-point configuration
L1 = kinematics.BOTTOM_ARM_LENGTH_MM   # Shoulder-to-Elbow length (100mm default)
L2 = kinematics.TOP_ARM_LENGTH_MM      # Elbow-to-End-Effector length (64mm default)
TOP_MOUNTING_OFFSET_ANGLE = -math.pi / 2  # Offset applied to elbow servo

# Physical Servo ROM Limits (Degrees)
LIMIT_ANGLE_MIN = 0.0
LIMIT_ANGLE_MAX = 180.0

# Workspace Visualizer Bounds (mm)
X_MIN, X_MAX = -250, 250
Y_MIN, Y_MAX = -250, 250
Z_MIN, Z_MAX =    0, 300

# ── Elegant Dark Theme Palette ────────────────────────────────────────────────
BG          = ( 13,  17,  23)   # Deep space dark background
PANEL_BG    = ( 22,  27,  34)   # Elegant dark grey panel background
BORDER      = ( 48,  54,  61)   # Slate gray border
GRID        = ( 33,  38,  45)   # Subtle grid line color
ACCENT      = ( 88, 166, 255)   # Cool neon blue accent
WHITE       = (240, 246, 252)   # High-contrast primary text
GREY        = (139, 148, 158)   # Muted label color
PLAYHEAD    = (240, 107, 117)   # Vibrant red playhead indicator

# Safety Status Colors
COLOR_VALID     = ( 46, 164,  79)  # Premium emerald green
COLOR_ANGLE_ERR = (219, 109,  40)  # High-vis orange for joint limit violations
COLOR_REACH_ERR = (215,  58,  73)  # Bright crimson red for unreachable targets

# Visual Layout
TIMELINE_W  = 280
PAD         = 16
ROW_H       = 44
HEADER_H    = 42
BUTTON_H    = 36

# ── Robust Mathematical Inverse Kinematics Engine ─────────────────────────────

def solve_ik(x, y, z):
    """
    Solves 3D Inverse Kinematics by calling kinematics.py.
    Returns:
      angles: tuple of floats (base_deg, bottom_deg, top_deg)
      state: string ('VALID', 'ANGLE_LIMIT', 'UNREACHABLE')
      joints: dict of 2D projection coordinates for visualization
    """
    try:
        # kinematics.get_robot_angles_radians returns base, bottom, top in radians
        base_rad, bottom_rad, top_rad = kinematics.get_robot_angles_radians(x, y, z)
        
        # Convert to degrees
        base_deg = math.degrees(base_rad)
        bottom_deg = math.degrees(bottom_rad)
        top_deg = math.degrees(top_rad)
        
        # In-line planar projection along the base rotation baseline (maintains physical link lengths L1 and L2):
        inverted_x = math.atan2(y, x) < 0
        r = math.sqrt(x**2 + y**2) * (-1.0 if inverted_x else 1.0)
        
        elbow_x = L1 * math.cos(bottom_rad) * (-1.0 if inverted_x else 1.0)
        elbow_z = L1 * math.sin(bottom_rad)
        
        angles = (base_deg, bottom_deg, top_deg)
        
        # Rotational angle bounds validation
        for deg in angles:
            if not (LIMIT_ANGLE_MIN <= deg <= LIMIT_ANGLE_MAX):
                return angles, 'ANGLE_LIMIT', {
                    'elbow_x': elbow_x, 'elbow_z': elbow_z, 'end_x': r, 'end_z': z
                }
                
        return angles, 'VALID', {
            'elbow_x': elbow_x, 'elbow_z': elbow_z, 'end_x': r, 'end_z': z
        }
        
    except (ValueError, AssertionError, Exception):
        return (0.0, 0.0, 0.0), 'UNREACHABLE', {}


# ── Waypoint Data Model ───────────────────────────────────────────────────────

class Waypoint:
    _counter = 1
    def __init__(self, x=100.0, y=0.0, z=100.0, gripper=90.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.gripper = float(gripper)
        self.label = f"P{Waypoint._counter:02d}"
        Waypoint._counter += 1
        self.update_status()

    def update_status(self):
        self.angles, self.status, self.joints = solve_ik(self.x, self.y, self.z)

    def as_ints(self):
        return int(round(self.x)), int(round(self.y)), int(round(self.z)), int(round(self.gripper))


# ── App Main Loop & User Interface ────────────────────────────────────────────

class RobotArmUI:
    def __init__(self):
        pygame.init()
        self.W, self.H = 1180, 740
        self.screen = pygame.display.set_mode((self.W, self.H), pygame.RESIZABLE)
        pygame.display.set_caption("🦾 Robotic Arm Analytical Controller")

        # Fonts Setup
        self.font_sm   = pygame.font.SysFont("Helvetica Neue, Arial, sans-serif", 13)
        self.font_md   = pygame.font.SysFont("Helvetica Neue, Arial, sans-serif", 15)
        self.font_lg   = pygame.font.SysFont("Helvetica Neue, Arial, sans-serif", 18, bold=True)
        self.font_mono = pygame.font.SysFont("Menlo, Consolas, Courier New, monospace", 13)

        # Application State
        self.waypoints      = []
        self.selected       = None
        self.run_index      = None
        self.running        = False
        self.run_thread     = None
        self.status_msg     = "System initialized. Click views to set waypoints."
        self.status_ok      = True

        # Interactive Target Coordinates (World Space)
        self.cursor_x = 100.0
        self.cursor_y = 0.0
        self.cursor_z = 100.0
        self.cursor_gripper = 90.0

        # Dynamic visual projection coordinates (smoothing / simulation playback)
        self.arm_x = 100.0
        self.arm_y = 0.0
        self.arm_z = 100.0
        self.arm_gripper = 90.0

        # Dragging states
        self.dragging_point = None  # (index, 'xy'|'xz')
        self.tl_scroll      = 0
        self.edit           = None  # {'idx', 'field', 'buf'}

        self.clock = pygame.time.Clock()
        self._last_tick = time.time()

        # Pre-populate with safe reference waypoints
        self._add_waypoint_at(100.0, 0.0, 100.0)
        self._add_waypoint_at(80.0, 80.0, 120.0)

    def _add_waypoint_at(self, x, y, z, gripper=None):
        if gripper is None:
            gripper = self.cursor_gripper
        wp = Waypoint(x, y, z, gripper)
        self.waypoints.append(wp)
        self.selected = len(self.waypoints) - 1

    # ── Geometry & Transform Helpers ──────────────────────────────────────────

    def _view_rects(self):
        W, H = self.screen.get_size()
        rx = TIMELINE_W + PAD
        ry = PAD + 60
        rw = W - TIMELINE_W - PAD * 2
        rh = (H - ry - PAD * 2) // 2
        xy_rect = pygame.Rect(rx, ry,          rw, rh)
        xz_rect = pygame.Rect(rx, ry + rh + PAD, rw, H - ry - rh - PAD * 2)
        return xy_rect, xz_rect

    def _w2xy(self, wx, wy, rect):
        px = rect.x + (wx - X_MIN) / (X_MAX - X_MIN) * rect.w
        py = rect.y + (1 - (wy - Y_MIN) / (Y_MAX - Y_MIN)) * rect.h
        return int(px), int(py)

    def _w2xz(self, wx, wz, rect):
        px = rect.x + (wx - X_MIN) / (X_MAX - X_MIN) * rect.w
        pz = rect.y + (1 - (wz - Z_MIN) / (Z_MAX - Z_MIN)) * rect.h
        return int(px), int(pz)

    def _xy2w(self, px, py, rect):
        return (X_MIN + (px - rect.x) / rect.w * (X_MAX - X_MIN),
                Y_MIN + (1 - (py - rect.y) / rect.h) * (Y_MAX - Y_MIN))

    def _xz2w(self, px, pz, rect):
        return (X_MIN + (px - rect.x) / rect.w * (X_MAX - X_MIN),
                Z_MIN + (1 - (pz - rect.y) / rect.h) * (Z_MAX - Z_MIN))

    def _clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def _px_per_mm(self, rect):
        return rect.w / (X_MAX - X_MIN)

    # ── Rendering Components ──────────────────────────────────────────────────

    def _draw_grid(self, rect, label_x, label_y, x_range, y_range):
        # Draw background panel
        pygame.draw.rect(self.screen, PANEL_BG, rect, border_radius=6)
        pygame.draw.rect(self.screen, BORDER, rect, 1, border_radius=6)

        # Draw grid lines
        steps = 6
        for i in range(steps + 1):
            t = i / steps
            gx = rect.x + int(t * rect.w)
            pygame.draw.line(self.screen, GRID, (gx, rect.y), (gx, rect.bottom))
            v = x_range[0] + t * (x_range[1] - x_range[0])
            lb = self.font_sm.render(f"{v:.0f}", True, GREY)
            self.screen.blit(lb, (gx - lb.get_width()//2, rect.bottom - lb.get_height() - 4))

            gy = rect.y + int(t * rect.h)
            pygame.draw.line(self.screen, GRID, (rect.x, gy), (rect.right, gy))
            v2 = y_range[1] - t * (y_range[1] - y_range[0])
            lb2 = self.font_sm.render(f"{v2:.0f}", True, GREY)
            self.screen.blit(lb2, (rect.x + 6, gy - lb2.get_height()//2))

        # Render reference axis markers
        ox, oy = self._w2xy(0, 0, rect) if label_y == 'Y' else self._w2xz(0, 0, rect)
        pygame.draw.line(self.screen, BORDER, (ox, rect.y), (ox, rect.bottom), 1)
        pygame.draw.line(self.screen, BORDER, (rect.x, oy), (rect.right, oy), 1)

    def _draw_reach_regions(self, rect, view):
        """Draws clear, shaded overlay areas representing reachability boundaries."""
        ppm = self._px_per_mm(rect)
        origin = self._w2xy(0, 0, rect) if view == 'xy' else self._w2xz(0, 0, rect)
        
        r_max = int((L1 + L2) * ppm)
        r_min = int(abs(L1 - L2) * ppm)
        
        # Max reach circle
        pygame.draw.circle(self.screen, (38, 48, 62), origin, r_max, 1)
        # Min reach circle
        pygame.draw.circle(self.screen, (38, 48, 62), origin, r_min, 1)

        # Label indicators
        lbl_max = self.font_sm.render(f"Max: {L1+L2:.0f}mm", True, BORDER)
        self.screen.blit(lbl_max, (origin[0] + r_max - 85, origin[1] + 4))
        lbl_min = self.font_sm.render(f"Min: {abs(L1-L2):.0f}mm", True, BORDER)
        self.screen.blit(lbl_min, (origin[0] + r_min + 6, origin[1] + 4))

    def _draw_robot_arm_xy(self, rect, tx, ty, status, joints):
        """Renders the top view cylindrical projection of base segment rotation."""
        origin = self._w2xy(0, 0, rect)
        end = self._w2xy(tx, ty, rect)
        
        # Base Line representation
        col = COLOR_VALID
        if status == 'UNREACHABLE':
            col = COLOR_REACH_ERR
        elif status == 'ANGLE_LIMIT':
            col = COLOR_ANGLE_ERR

        # Draw main arm outline top projection
        pygame.draw.line(self.screen, col, origin, end, 6)
        pygame.draw.circle(self.screen, WHITE, origin, 8)
        pygame.draw.circle(self.screen, WHITE, end, 6)

    def _draw_robot_arm_xz(self, rect, tx, tz, status, joints):
        """Renders 2-link planar structure in the vertical projection."""
        origin = self._w2xz(0, 0, rect)
        
        col = COLOR_VALID
        if status == 'UNREACHABLE':
            col = COLOR_REACH_ERR
            # If unreachable, draw simple straight dashed error line
            end = self._w2xz(tx, tz, rect)
            pygame.draw.line(self.screen, col, origin, end, 2)
            pygame.draw.circle(self.screen, col, end, 6)
            return

        if status == 'ANGLE_LIMIT':
            col = COLOR_ANGLE_ERR

        # Retrieve mapped coordinates from IK joints dictionary
        ex = joints.get('elbow_x', 0)
        ez = joints.get('elbow_z', L1)
        rx = joints.get('end_x', tx)
        rz = joints.get('end_z', tz)
        
        elbow = self._w2xz(ex, ez, rect)
        end = self._w2xz(rx, rz, rect)
        
        # Draw physical link geometries
        pygame.draw.line(self.screen, (150, 160, 175), origin, elbow, 8)  # Lower Arm link
        pygame.draw.line(self.screen, (110, 125, 140), elbow, end, 5)     # Upper Arm link
        
        # Draw joints circles
        pygame.draw.circle(self.screen, BORDER, origin, 10)
        pygame.draw.circle(self.screen, WHITE, elbow, 6)
        pygame.draw.circle(self.screen, col, end, 8)
        pygame.draw.circle(self.screen, WHITE, end, 3)

    def _draw_waypoints_and_path(self, view, rect):
        if len(self.waypoints) < 1:
            return
            
        pts = []
        for i, wp in enumerate(self.waypoints):
            if view == 'xy':
                p = self._w2xy(wp.x, wp.y, rect)
            else:
                wp_inverted = math.atan2(wp.y, wp.x) < 0
                wp_r = math.sqrt(wp.x**2 + wp.y**2) * (-1.0 if wp_inverted else 1.0)
                p = self._w2xz(wp_r, wp.z, rect)
            pts.append(p)
            
            # Map safety color
            c = COLOR_VALID
            if wp.status == 'UNREACHABLE':
                c = COLOR_REACH_ERR
            elif wp.status == 'ANGLE_LIMIT':
                c = COLOR_ANGLE_ERR

            # Select highlighting
            if i == self.selected:
                pygame.draw.circle(self.screen, WHITE, p, 10, 2)
            if i == self.run_index and self.running:
                pygame.draw.circle(self.screen, (255, 255, 255), p, 11)

            pygame.draw.circle(self.screen, c, p, 7)
            pygame.draw.circle(self.screen, BG, p, 5)
            pygame.draw.circle(self.screen, c, p, 3)

            # Node label text
            lbl = self.font_sm.render(wp.label, True, WHITE)
            self.screen.blit(lbl, (p[0] + 10, p[1] - 8))

        # Render paths lines connecting waypoints
        if len(pts) > 1:
            for i in range(len(pts) - 1):
                pygame.draw.line(self.screen, ACCENT, pts[i], pts[i+1], 1)

    def _draw_timeline(self, rect):
        pygame.draw.rect(self.screen, PANEL_BG, rect, border_radius=6)
        pygame.draw.rect(self.screen, BORDER, rect, 1, border_radius=6)

        # Title
        t = self.font_lg.render("WAYPOINTS TIMELINE", True, WHITE)
        self.screen.blit(t, (rect.x + PAD, rect.y + 12))

        # Add / Delete / Clear Buttons layout
        by = rect.y + HEADER_H + 4
        self.btn_add    = self._draw_btn("+ ADD",   rect.x+PAD,          by,         85, BUTTON_H, COLOR_VALID)
        self.btn_delete = self._draw_btn("DEL",     rect.x+PAD+93,       by,         54, BUTTON_H, COLOR_REACH_ERR)
        self.btn_clear  = self._draw_btn("CLEAR",   rect.x+PAD+93+62,    by,         70, BUTTON_H, (100,30,40))
        
        run_lbl = "■ STOP SEQUENCE" if self.running else "▶ RUN TIMELINE"
        run_col = COLOR_REACH_ERR if self.running else ACCENT
        self.btn_run = self._draw_btn(run_lbl, rect.x+PAD, by + BUTTON_H + 8, rect.w - PAD*2, BUTTON_H, run_col)

        # Waypoint list container with vertical clipping
        list_y = by + BUTTON_H*2 + 20
        list_rect = pygame.Rect(rect.x, list_y, rect.w, rect.bottom - list_y - 30)
        self.screen.set_clip(list_rect)

        for i, wp in enumerate(self.waypoints):
            ry = list_y + i * ROW_H - self.tl_scroll
            if ry + ROW_H < list_y or ry > rect.bottom:
                continue
                
            row_rect = pygame.Rect(rect.x + 8, ry, rect.w - 16, ROW_H - 4)
            
            # Select / active execution states
            bg_col = BG
            if i == self.selected:
                bg_col = (29, 35, 48)
            if i == self.run_index and self.running:
                bg_col = (20, 50, 32)
                
            pygame.draw.rect(self.screen, bg_col, row_rect, border_radius=4)
            pygame.draw.rect(self.screen, BORDER if i != self.selected else ACCENT, row_rect, 1, border_radius=4)

            # Node name
            n_lbl = self.font_md.render(wp.label, True, WHITE)
            self.screen.blit(n_lbl, (row_rect.x + 8, ry + 4))

            # Status visual indicator labels
            stat_str = "Valid"
            stat_c = COLOR_VALID
            if wp.status == 'UNREACHABLE':
                stat_str = "⚠️ Out of Reach"
                stat_c = COLOR_REACH_ERR
            elif wp.status == 'ANGLE_LIMIT':
                stat_str = "⚠️ Limit Exceeded"
                stat_c = COLOR_ANGLE_ERR
                
            s_lbl = self.font_sm.render(stat_str, True, stat_c)
            self.screen.blit(s_lbl, (row_rect.right - s_lbl.get_width() - 8, ry + 4))

            # Coordinates string
            coord_str = f"X: {wp.x:+.1f} | Y: {wp.y:+.1f} | Z: {wp.z:+.1f} | G: {wp.gripper:.0f}°"
            c_lbl = self.font_mono.render(coord_str, True, GREY)
            self.screen.blit(c_lbl, (row_rect.x + 8, ry + 20))

            if i == self.run_index and self.running:
                pygame.draw.circle(self.screen, PLAYHEAD, (row_rect.right - 12, row_rect.centery + 10), 4)

        self.screen.set_clip(None)

        # Inline XYZ Coordinate Editor Panel
        if self.edit is not None and self.selected is not None:
            self._draw_editor_box(rect, list_y)

        # Status strip at bottom
        st_c = COLOR_VALID if self.status_ok else COLOR_REACH_ERR
        st_lbl = self.font_sm.render(self.status_msg, True, st_c)
        self.screen.blit(st_lbl, (rect.x + PAD, rect.bottom - 22))

    def _draw_btn(self, text, x, y, w, h, base_color):
        r = pygame.Rect(x, y, w, h)
        hover = r.collidepoint(pygame.mouse.get_pos())
        c = tuple(min(255, v+30) for v in base_color) if hover else base_color
        
        pygame.draw.rect(self.screen, c, r, border_radius=4)
        pygame.draw.rect(self.screen, BORDER, r, 1, border_radius=4)
        
        txt = self.font_md.render(text, True, BG if hover else WHITE)
        self.screen.blit(txt, (r.centerx - txt.get_width()//2, r.centery - txt.get_height()//2))
        return r

    def _draw_editor_box(self, tl_rect, list_y):
        wp = self.waypoints[self.selected]
        ey = list_y + self.selected * ROW_H - self.tl_scroll
        ey = max(list_y + 2, ey)
        ex = tl_rect.x + PAD
        
        panel = pygame.Rect(ex - 4, ey - 2, tl_rect.w - PAD*2 + 8, 80)
        pygame.draw.rect(self.screen, (15, 20, 30), panel, border_radius=6)
        pygame.draw.rect(self.screen, ACCENT, panel, 1, border_radius=6)

        fields = [('x', wp.x), ('y', wp.y), ('z', wp.z), ('gripper', wp.gripper)]
        for idx, (field, val) in enumerate(fields):
            fx = ex + idx * 60
            fy = ey + 8
            active = self.edit['field'] == field
            
            box_rect = pygame.Rect(fx, fy, 56, 28)
            pygame.draw.rect(self.screen, PANEL_BG if not active else (22, 40, 70), box_rect, border_radius=4)
            pygame.draw.rect(self.screen, ACCENT if active else BORDER, box_rect, 1, border_radius=4)
            
            buf_str = self.edit['buf'] if active else f"{val:.1f}"
            caret = "|" if active and (pygame.time.get_ticks() // 500) % 2 == 0 else ""
            
            label_text = f"{'G' if field=='gripper' else field.upper()}:{buf_str}{caret}"
            t_lbl = self.font_mono.render(label_text, True, WHITE if active else GREY)
            self.screen.blit(t_lbl, (fx + 4, fy + 7))

        hint = self.font_sm.render("Tab: Next  |  Enter: Save  |  Esc: Cancel", True, GREY)
        self.screen.blit(hint, (ex, ey + 42))

    def _draw_selected_hud(self):
        """Displays real-time kinematic debugging specs for selected waypoint in a dedicated header bar."""
        W, H = self.screen.get_size()
        
        # Draw subtle bottom border/separator line for the HUD bar
        pygame.draw.line(self.screen, BORDER, (TIMELINE_W + PAD, PAD + 48), (W - PAD, PAD + 48), 1)
        
        if self.selected is None or self.selected >= len(self.waypoints):
            # Render a premium fallback message when no waypoint is selected
            msg = self.font_md.render("No Waypoint Selected — Click anywhere on the grids to add or edit waypoints", True, GREY)
            self.screen.blit(msg, (TIMELINE_W + PAD + 10, PAD + 14))
            return
            
        wp = self.waypoints[self.selected]
        
        # Color coding state
        col = COLOR_VALID
        stat_lbl = "VALID"
        if wp.status == 'UNREACHABLE':
            col = COLOR_REACH_ERR
            stat_lbl = "UNREACHABLE (OUT OF PHYSICAL RADIUS)"
        elif wp.status == 'ANGLE_LIMIT':
            col = COLOR_ANGLE_ERR
            stat_lbl = "ANGLE BOUND EXCEEDED (LIMITS: 0° - 180°)"

        info_text = f"SELECTED: {wp.label} | STATE: {stat_lbl}"
        deg_text = f"Base: {wp.angles[0]:.1f}° | Shoulder: {wp.angles[1]:.1f}° | Elbow: {wp.angles[2]:.1f}° | Gripper: {wp.gripper:.1f}°"
        
        lbl_info = self.font_lg.render(info_text, True, col)
        lbl_deg = self.font_mono.render(deg_text, True, GREY)
        
        # Render within the dedicated HUD vertical safe space
        self.screen.blit(lbl_info, (TIMELINE_W + PAD + 10, PAD))
        self.screen.blit(lbl_deg, (TIMELINE_W + PAD + 10, PAD + 24))

    # ── Threaded Serial Sequence Executor (Resolving Reset Bugs) ───────────────

    def _start_run(self):
        if not self.waypoints:
            self.status_msg = "No waypoints configured."; self.status_ok = False; return
        if self.running:
            return
            
        self.running = True
        self.status_ok = True
        self.run_thread = threading.Thread(target=self._run_sequence, daemon=True)
        self.run_thread.start()

    def _stop_run(self):
        self.running = False
        self.run_index = None
        self.status_msg = "Sequence execution aborted."
        self.status_ok = True

    def _run_sequence(self):
        if USE_REAL_ARM:
            try:
                self.status_msg = "Connecting to serial controller..."
                # Open connection and retain lock during the entire execution loop
                with serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, timeout=1.0) as ser:
                    # Allow Arduino bootloader to initialize after DTR line reset
                    
                    for i, wp in enumerate(self.waypoints):
                        if not self.running:
                            break
                        
                        self.run_index = i
                        self.status_msg = f"Moving to {wp.label}..."
                        
                        # Set active visuals to target values
                        self.arm_x = wp.x
                        self.arm_y = wp.y
                        self.arm_z = wp.z
                        self.arm_gripper = wp.gripper
                        
                        # Transmit coordinates using the official movement API in go_to.py
                        go_to.go_to_radians(ser, wp.x, wp.y, wp.z, wp.gripper)
                        
                        # Realistic physical travel time delay (allows servos to sweep safely)
                        time.sleep(10)
                        
            except Exception as e:
                self.status_msg = f"Serial Error: {str(e)}"
                self.status_ok = False
                self.running = False
                self.run_index = None
                return
        else:
            # Simulation playback execution path
            for i, wp in enumerate(self.waypoints):
                if not self.running:
                    break
                self.run_index = i
                self.status_msg = f"[SIMULATION] Moving to {wp.label}..."
                
                # Smooth animated transition to target
                t0 = time.time()
                while time.time() - t0 < 1.5:
                    if not self.running:
                        break
                    dx = wp.x - self.arm_x
                    dy = wp.y - self.arm_y
                    dz = wp.z - self.arm_z
                    
                    step = 100.0 * 0.05  # simulation mm/s step
                    dist = math.hypot(math.hypot(dx, dy), dz)
                    
                    if dist <= step:
                        self.arm_x, self.arm_y, self.arm_z = wp.x, wp.y, wp.z
                        self.arm_gripper = wp.gripper
                        break
                    else:
                        ratio = step / dist
                        self.arm_x += dx * ratio
                        self.arm_y += dy * ratio
                        self.arm_z += dz * ratio
                        self.arm_gripper += (wp.gripper - self.arm_gripper) * ratio
                    time.sleep(0.05)
                
                self.arm_x, self.arm_y, self.arm_z = wp.x, wp.y, wp.z
                self.arm_gripper = wp.gripper
                time.sleep(0.2)

        self.running = False
        self.run_index = None
        self.status_msg = "Timeline sequence complete ✓"
        self.status_ok = True

    # ── Event Processing ──────────────────────────────────────────────────────

    def _handle_key(self, event):
        if self.edit is not None:
            if event.key == pygame.K_RETURN:
                self._commit_edit()
            elif event.key == pygame.K_ESCAPE:
                self.edit = None
            elif event.key == pygame.K_TAB:
                self._commit_edit()
                fields = ['x', 'y', 'z', 'gripper']
                next_f = fields[(fields.index(self.edit['field']) + 1) % 4]
                wp = self.waypoints[self.selected]
                self.edit = {
                    'idx': self.selected,
                    'field': next_f,
                    'buf': f"{getattr(wp, next_f):.1f}"
                }
            elif event.key == pygame.K_BACKSPACE:
                self.edit['buf'] = self.edit['buf'][:-1]
            elif event.unicode in "0123456789.-":
                self.edit['buf'] += event.unicode
        else:
            if event.key in (pygame.K_DELETE, pygame.K_BACKSPACE):
                self._delete_selected()
            elif event.key == pygame.K_a:
                self._add_waypoint_at(self.cursor_x, self.cursor_y, self.cursor_z)
            elif event.key == pygame.K_RETURN:
                self._start_run()
            elif event.key == pygame.K_ESCAPE:
                self._stop_run()
            elif event.key == pygame.K_UP and self.selected is not None:
                self.selected = max(0, self.selected - 1)
            elif event.key == pygame.K_DOWN and self.selected is not None:
                self.selected = min(len(self.waypoints) - 1, self.selected + 1)

    def _commit_edit(self):
        if not self.edit:
            return
        try:
            val = float(self.edit['buf'])
            wp = self.waypoints[self.edit['idx']]
            field = self.edit['field']
            
            # Workspace clamping parameters
            limits = {
                'x': (X_MIN, X_MAX),
                'y': (Y_MIN, Y_MAX),
                'z': (Z_MIN, Z_MAX),
                'gripper': (LIMIT_ANGLE_MIN, LIMIT_ANGLE_MAX)
            }[field]
            setattr(wp, field, self._clamp(val, *limits))
            wp.update_status()
            
            self.status_msg = f"Updated {wp.label} {field.upper()} -> {val:.1f}"
            self.status_ok = True
        except ValueError:
            pass
        self.edit = None

    def _delete_selected(self):
        if self.selected is not None and self.waypoints:
            wp = self.waypoints.pop(self.selected)
            self.selected = min(self.selected, len(self.waypoints) - 1) if self.waypoints else None
            self.edit = None
            self.status_msg = f"Removed {wp.label}."
            self.status_ok = True

    def _click_view(self, mx, my, rect, view):
        # 1. Check if user clicked to drag an existing waypoint first
        for i, wp in enumerate(self.waypoints):
            if view == 'xy':
                p = self._w2xy(wp.x, wp.y, rect)
            else:
                wp_inverted = math.atan2(wp.y, wp.x) < 0
                wp_r = math.sqrt(wp.x**2 + wp.y**2) * (-1.0 if wp_inverted else 1.0)
                p = self._w2xz(wp_r, wp.z, rect)
                
            if math.hypot(mx - p[0], my - p[1]) < 12:
                self.selected = i
                self.dragging_point = (i, view)
                self.edit = None
                return
                
        # 2. Clicked empty workspace space -> update cursor position and add waypoint
        if view == 'xy':
            cx, cy = self._xy2w(mx, my, rect)
            self.cursor_x = self._clamp(cx, X_MIN, X_MAX)
            self.cursor_y = self._clamp(cy, Y_MIN, Y_MAX)
        else:
            cr, cz = self._xz2w(mx, my, rect)
            cr = self._clamp(cr, X_MIN, X_MAX)
            self.cursor_z = self._clamp(cz, Z_MIN, Z_MAX)
            # Maintain the current base rotation baseline angle in [0, pi]
            beta = math.atan2(self.cursor_y, self.cursor_x)
            if beta < 0:
                beta += math.pi
            r = cr * (1.0 if math.cos(beta) >= 0 else -1.0)
            self.cursor_x = r * math.cos(beta)
            self.cursor_y = r * math.sin(beta)
            
        self._add_waypoint_at(self.cursor_x, self.cursor_y, self.cursor_z)
        self.dragging_point = (len(self.waypoints) - 1, view)

    def _click_timeline(self, mx, my, tl_rect):
        by = tl_rect.y + HEADER_H + 4
        list_y = by + BUTTON_H*2 + 20
        for i in range(len(self.waypoints)):
            ry = list_y + i * ROW_H - self.tl_scroll
            rr = pygame.Rect(tl_rect.x + 8, ry, tl_rect.w - 16, ROW_H - 4)
            if rr.collidepoint(mx, my):
                if self.selected == i:
                    # Double click equivalent: open editor
                    wp = self.waypoints[i]
                    self.edit = {'idx': i, 'field': 'x', 'buf': f"{wp.x:.1f}"}
                else:
                    self.selected = i
                    self.edit = None
                return

    def run(self):
        global USE_REAL_ARM
        if USE_REAL_ARM:
            # Test import standard serial module silently
            try:
                import serial
            except ImportError:
                print("PySerial package is missing. Switching to Simulation Mode.")
                USE_REAL_ARM = False

        while True:
            dt = self.clock.tick(60) / 1000.0

            # Dynamic tracking loops
            if not self.running:
                # Follow live cursor coordinates
                self.arm_x = self.cursor_x
                self.arm_y = self.cursor_y
                self.arm_z = self.cursor_z
                self.arm_gripper = self.cursor_gripper

            # Calculate active IK solutions for drawing
            arm_angles, arm_status, arm_joints = solve_ik(self.arm_x, self.arm_y, self.arm_z)

            # Processing UI events
            for event in pygame.event.get():
                xy_rect, xz_rect = self._view_rects()
                W, H = self.screen.get_size()
                tl_rect = pygame.Rect(0, 0, TIMELINE_W, H)

                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                    
                elif event.type == pygame.VIDEORESIZE:
                    self.W, self.H = event.size
                    self.screen = pygame.display.set_mode((self.W, self.H), pygame.RESIZABLE)
                    
                elif event.type == pygame.KEYDOWN:
                    self._handle_key(event)
                    
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    mx, my = event.pos
                    if self.btn_add and self.btn_add.collidepoint(event.pos):
                        self._add_waypoint_at(self.cursor_x, self.cursor_y, self.cursor_z)
                    elif self.btn_delete and self.btn_delete.collidepoint(event.pos):
                        self._delete_selected()
                    elif self.btn_clear and self.btn_clear.collidepoint(event.pos):
                        self.waypoints.clear()
                        self.selected = None
                        self.tl_scroll = 0
                        self.status_msg = "Timeline cleared."
                        self.status_ok = True
                    elif self.btn_run and self.btn_run.collidepoint(event.pos):
                        self._stop_run() if self.running else self._start_run()
                    elif xy_rect.collidepoint(mx, my):
                        self._click_view(mx, my, xy_rect, 'xy')
                    elif xz_rect.collidepoint(mx, my):
                        self._click_view(mx, my, xz_rect, 'xz')
                    elif tl_rect.collidepoint(mx, my):
                        self._click_timeline(mx, my, tl_rect)
                        
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    # Handle timeline list mouse wheel scroll
                    if tl_rect.collidepoint(event.pos):
                        if event.button == 4:  # Wheel up
                            self.tl_scroll = max(0, self.tl_scroll - ROW_H)
                        elif event.button == 5:  # Wheel down
                            max_s = max(0, len(self.waypoints) * ROW_H - (H - list_y - 40))
                            self.tl_scroll = min(max_s, self.tl_scroll + ROW_H)
                            
                elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                    self.dragging_point = None
                    
                elif event.type == pygame.MOUSEMOTION:
                    mx, my = event.pos
                    
                    # Live tracking updates
                    if xy_rect.collidepoint(mx, my) and self.dragging_point is None:
                        cx, cy = self._xy2w(mx, my, xy_rect)
                        self.cursor_x = self._clamp(cx, X_MIN, X_MAX)
                        self.cursor_y = self._clamp(cy, Y_MIN, Y_MAX)
                    elif xz_rect.collidepoint(mx, my) and self.dragging_point is None:
                        cr, cz = self._xz2w(mx, my, xz_rect)
                        cr = self._clamp(cr, X_MIN, X_MAX)
                        self.cursor_z = self._clamp(cz, Z_MIN, Z_MAX)
                        # Maintain current base rotation baseline angle in [0, pi]
                        beta = math.atan2(self.cursor_y, self.cursor_x)
                        if beta < 0:
                            beta += math.pi
                        r = cr * (1.0 if math.cos(beta) >= 0 else -1.0)
                        self.cursor_x = r * math.cos(beta)
                        self.cursor_y = r * math.sin(beta)
                        
                    # Handle Waypoint Dragging dynamics
                    if self.dragging_point is not None:
                        idx, view = self.dragging_point
                        wp = self.waypoints[idx]
                        if view == 'xy' and xy_rect.collidepoint(mx, my):
                            wp.x, wp.y = self._xy2w(mx, my, xy_rect)
                            wp.x = self._clamp(wp.x, X_MIN, X_MAX)
                            wp.y = self._clamp(wp.y, Y_MIN, Y_MAX)
                            # Update cursor and active arm position to follow the dragged point
                            self.cursor_x = wp.x
                            self.cursor_y = wp.y
                        elif view == 'xz' and xz_rect.collidepoint(mx, my):
                            cr, cz = self._xz2w(mx, my, xz_rect)
                            cr = self._clamp(cr, X_MIN, X_MAX)
                            cz = self._clamp(cz, Z_MIN, Z_MAX)
                            # Maintain current waypoint base rotation angle in [0, pi]
                            wp_beta = math.atan2(wp.y, wp.x)
                            if wp_beta < 0:
                                wp_beta += math.pi
                            wp_r = cr * (1.0 if math.cos(wp_beta) >= 0 else -1.0)
                            wp.x = wp_r * math.cos(wp_beta)
                            wp.y = wp_r * math.sin(wp_beta)
                            wp.z = cz
                            # Update cursor and active arm position to follow the dragged point
                            self.cursor_x = wp.x
                            self.cursor_y = wp.y
                            self.cursor_z = wp.z
                        wp.update_status()

            # Render Pipeline
            self.screen.fill(BG)
            
            W, H = self.screen.get_size()
            tl_rect  = pygame.Rect(0, 0, TIMELINE_W, H)
            xy_rect, xz_rect = self._view_rects()

            # ── XY Projection Grid (Top View) ─────────────────────────────────
            self._draw_grid(xy_rect, 'X', 'Y', (X_MIN, X_MAX), (Y_MIN, Y_MAX))
            self._draw_reach_regions(xy_rect, 'xy')
            
            # Render Ghost arm geometries at each waypoint
            for wp in self.waypoints:
                self._draw_robot_arm_xy(xy_rect, wp.x, wp.y, wp.status, wp.joints)
                
            self._draw_robot_arm_xy(xy_rect, self.arm_x, self.arm_y, arm_status, arm_joints)
            self._draw_waypoints_and_path('xy', xy_rect)
            
            # Cursor crosshair lines
            cp_xy = self._w2xy(self.cursor_x, self.cursor_y, xy_rect)
            pygame.draw.line(self.screen, GRID, (cp_xy[0], xy_rect.y), (cp_xy[0], xy_rect.bottom), 1)
            pygame.draw.line(self.screen, GRID, (xy_rect.x, cp_xy[1]), (xy_rect.right, cp_xy[1]), 1)
            
            # View Labels
            lbl_xy = self.font_lg.render("XY TOP VIEW (BASE ROTATION)", True, ACCENT)
            self.screen.blit(lbl_xy, (xy_rect.x + 12, xy_rect.y + 10))

            # ── XZ Projection Grid (Side/Front View) ──────────────────────────
            self._draw_grid(xz_rect, 'X', 'Z', (X_MIN, X_MAX), (Z_MIN, Z_MAX))
            self._draw_reach_regions(xz_rect, 'xz')
            
            for wp in self.waypoints:
                wp_inverted = math.atan2(wp.y, wp.x) < 0
                wp_r = math.sqrt(wp.x**2 + wp.y**2) * (-1.0 if wp_inverted else 1.0)
                self._draw_robot_arm_xz(xz_rect, wp_r, wp.z, wp.status, wp.joints)
                
            arm_inverted = math.atan2(self.arm_y, self.arm_x) < 0
            arm_r = math.sqrt(self.arm_x**2 + self.arm_y**2) * (-1.0 if arm_inverted else 1.0)
            self._draw_robot_arm_xz(xz_rect, arm_r, self.arm_z, arm_status, arm_joints)
            self._draw_waypoints_and_path('xz', xz_rect)
            
            # Cursor crosshair lines
            cursor_inverted = math.atan2(self.cursor_y, self.cursor_x) < 0
            cursor_r = math.sqrt(self.cursor_x**2 + self.cursor_y**2) * (-1.0 if cursor_inverted else 1.0)
            cp_xz = self._w2xz(cursor_r, self.cursor_z, xz_rect)
            pygame.draw.line(self.screen, GRID, (cp_xz[0], xz_rect.y), (cp_xz[0], xz_rect.bottom), 1)
            pygame.draw.line(self.screen, GRID, (xz_rect.x, cp_xz[1]), (xz_rect.right, cp_xz[1]), 1)
            
            lbl_xz = self.font_lg.render("XZ FRONT VIEW (PLANAR MOVEMENT)", True, ACCENT)
            self.screen.blit(lbl_xz, (xz_rect.x + 12, xz_rect.y + 10))

            # ── Timeline Sidebar Panel ────────────────────────────────────────
            self._draw_timeline(tl_rect)
            self._draw_selected_hud()

            pygame.display.flip()


if __name__ == "__main__":
    RobotArmUI().run()
