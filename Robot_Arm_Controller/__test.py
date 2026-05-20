import pygame
import sys
import math
from math import sin, cos, sqrt, acos, atan, degrees, radians

# ─── Kinematics (faithful copy of provided code, with typo fix noted) ──────────

lower_arm_length = 100
upper_arm_length = 64

def arm_x_y_kinematics(x, y):
    if x == 0: x = 1
    if y == 0: y = 1
    dist = sqrt(x**2 + y**2)
    if dist > 164 or dist < 36:
        return None
    upper = acos((x**2 + y**2 - lower_arm_length**2 - upper_arm_length**2)
                 / (2 * lower_arm_length * upper_arm_length))
    lower = atan(y / x) - atan(upper_arm_length * sin(upper)
                                / (lower_arm_length + upper_arm_length * cos(upper)))
    return (degrees(upper), degrees(lower))

def arm_z_x_kinematics(x, z):
    if x == 0: x = 1
    if z == 0: z = 1   # NOTE: original code has `y = 1` here (typo) — corrected to z
    dist = sqrt(x**2 + z**2)
    if dist > 164 or dist < 36:
        return None
    return atan(z / x)

def arm_full_kinematics(x, y, z):
    base_angle = arm_z_x_kinematics(x, z)
    if base_angle is None:
        return None
    arm_x = sqrt(z**2 + x**2)
    res = arm_x_y_kinematics(arm_x, y)
    if res is None:
        return None
    upper_arm_angle, lower_arm_angle_raw = res
    lower_arm_angle = 180 - lower_arm_angle_raw
    if lower_arm_angle < 180 and upper_arm_angle < 180 and base_angle < 180:
        return lower_arm_angle, upper_arm_angle, base_angle
    return None

# ─── Constants ─────────────────────────────────────────────────────────────────

W, H   = 1100, 680
FPS    = 60
SCALE  = 1.6

BG        = (10, 12, 18)
GRID      = (22, 28, 40)
PANEL_BG  = (14, 18, 28)
ACCENT    = (0, 220, 180)
ACCENT2   = (255, 180, 0)
DIM       = (60, 75, 100)
WHITE     = (220, 230, 245)
RED       = (220, 60, 80)
GREEN     = (80, 220, 120)
LINK1_COL = (0, 200, 255)
LINK2_COL = (255, 120, 60)
BASE_COL  = (180, 200, 220)

# ─── Drawing helpers ───────────────────────────────────────────────────────────

def draw_glow_circle(surf, color, pos, r, alpha=60):
    s = pygame.Surface((r * 4, r * 4), pygame.SRCALPHA)
    for i in range(3, 0, -1):
        pygame.draw.circle(s, (*color, alpha // i), (r * 2, r * 2), r * i)
    surf.blit(s, (pos[0] - r * 2, pos[1] - r * 2))

def draw_glow_line(surf, color, p1, p2, width=2, alpha=80):
    s = pygame.Surface((W, H), pygame.SRCALPHA)
    pygame.draw.line(s, (*color, alpha), p1, p2, width + 4)
    pygame.draw.line(s, (*color, 200), p1, p2, width)
    surf.blit(s, (0, 0))

def draw_grid(surf, rect, spacing=20):
    x0, y0, w, h = rect
    for x in range(x0, x0 + w, spacing):
        pygame.draw.line(surf, GRID, (x, y0), (x, y0 + h))
    for y in range(y0, y0 + h, spacing):
        pygame.draw.line(surf, GRID, (x0, y), (x0 + w, y))

def draw_panel_border(surf, rect, color=ACCENT, alpha=120):
    s = pygame.Surface((W, H), pygame.SRCALPHA)
    pygame.draw.rect(s, (*color, alpha), rect, 1)
    surf.blit(s, (0, 0))

def world_to_vp(vx, vy, cx, cy):
    return cx + int(vx * SCALE), cy - int(vy * SCALE)

def vp_to_world(px, py, cx, cy):
    return (px - cx) / SCALE, -(py - cy) / SCALE

# ─── Arm renders ───────────────────────────────────────────────────────────────

def draw_arm_xy(surf, ox, oy, vw, vh, mx, my, font_label):
    cx = ox + vw // 2
    cy = oy + vh // 2 + 20

    for r, col in [(164 * SCALE, DIM), (36 * SCALE, DIM), (100 * SCALE, GRID)]:
        pygame.draw.circle(surf, col, (cx, cy), int(r), 1)
    pygame.draw.line(surf, DIM, (ox + 10, cy), (ox + vw - 10, cy), 1)
    pygame.draw.line(surf, DIM, (cx, oy + 10), (cx, oy + vh - 10), 1)

    lx = font_label.render("X →", True, DIM)
    ly = font_label.render("↑ Y", True, DIM)
    surf.blit(lx, (ox + vw - 35, cy + 5))
    surf.blit(ly, (cx + 5, oy + 28))

    res = arm_x_y_kinematics(mx, my)
    if res is None:
        tx, ty = world_to_vp(mx, my, cx, cy)
        pygame.draw.circle(surf, RED, (tx, ty), 6, 2)
        return False, None, None

    upper_deg, lower_deg = res
    lower_rad = radians(lower_deg)
    upper_rad = radians(upper_deg)

    ex = cx + int(lower_arm_length * SCALE * cos(lower_rad))
    ey = cy - int(lower_arm_length * SCALE * sin(lower_rad))
    tx = ex + int(upper_arm_length * SCALE * cos(lower_rad + upper_rad))
    ty = ey - int(upper_arm_length * SCALE * sin(lower_rad + upper_rad))

    draw_glow_line(surf, LINK1_COL, (cx, cy), (ex, ey), 3, 60)
    draw_glow_line(surf, LINK2_COL, (ex, ey), (tx, ty), 3, 60)
    pygame.draw.line(surf, LINK1_COL, (cx, cy), (ex, ey), 3)
    pygame.draw.line(surf, LINK2_COL, (ex, ey), (tx, ty), 3)

    draw_glow_circle(surf, BASE_COL, (cx, cy), 8, 80)
    pygame.draw.circle(surf, BASE_COL, (cx, cy), 6)
    draw_glow_circle(surf, LINK1_COL, (ex, ey), 6, 80)
    pygame.draw.circle(surf, LINK1_COL, (ex, ey), 5)
    draw_glow_circle(surf, ACCENT, (tx, ty), 7, 100)
    pygame.draw.circle(surf, ACCENT, (tx, ty), 5)

    return True, upper_deg, lower_deg


def draw_arm_xz(surf, ox, oy, vw, vh, mx, mz, font_label):
    cx = ox + vw // 2
    cy = oy + vh // 2 + 20

    for r, col in [(164 * SCALE, DIM), (36 * SCALE, DIM), (100 * SCALE, GRID)]:
        pygame.draw.circle(surf, col, (cx, cy), int(r), 1)
    pygame.draw.line(surf, DIM, (ox + 10, cy), (ox + vw - 10, cy), 1)
    pygame.draw.line(surf, DIM, (cx, oy + 10), (cx, oy + vh - 10), 1)

    lx = font_label.render("X →", True, DIM)
    lz = font_label.render("↑ Z", True, DIM)
    surf.blit(lx, (ox + vw - 35, cy + 5))
    surf.blit(lz, (cx + 5, oy + 28))

    res = arm_z_x_kinematics(mx, mz)
    if res is None:
        tx, tz = world_to_vp(mx, mz, cx, cy)
        pygame.draw.circle(surf, RED, (tx, tz), 6, 2)
        return False, None

    base_rad = res
    arm_r = sqrt(mx**2 + mz**2)
    tp_x = cx + int(arm_r * SCALE * cos(base_rad))
    tp_z = cy - int(arm_r * SCALE * sin(base_rad))

    draw_glow_line(surf, ACCENT2, (cx, cy), (tp_x, tp_z), 3, 60)
    pygame.draw.line(surf, ACCENT2, (cx, cy), (tp_x, tp_z), 3)

    draw_glow_circle(surf, BASE_COL, (cx, cy), 8, 80)
    pygame.draw.circle(surf, BASE_COL, (cx, cy), 6)
    draw_glow_circle(surf, ACCENT, (tp_x, tp_z), 7, 100)
    pygame.draw.circle(surf, ACCENT, (tp_x, tp_z), 5)

    return True, degrees(base_rad)

# ─── Main ──────────────────────────────────────────────────────────────────────

def main():
    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("Robotic Arm Kinematics v2 — Control Panel")
    clock = pygame.time.Clock()

    try:
        font_title = pygame.font.SysFont("Courier New", 13, bold=True)
        font_label = pygame.font.SysFont("Courier New", 11)
        font_val   = pygame.font.SysFont("Courier New", 12, bold=True)
        font_big   = pygame.font.SysFont("Courier New", 20, bold=True)
        font_warn  = pygame.font.SysFont("Courier New", 10)
    except:
        font_title = pygame.font.SysFont(None, 14)
        font_label = pygame.font.SysFont(None, 12)
        font_val   = pygame.font.SysFont(None, 13)
        font_big   = pygame.font.SysFont(None, 22)
        font_warn  = pygame.font.SysFont(None, 11)

    target_x, target_y, target_z = 123.0, 100.0, 0.67   # matches print() in source

    VP_W, VP_H = 480, 500
    VP_XY = (30, 110)
    VP_XZ = (590, 110)

    dragging_xy = dragging_xz = False
    slider_dragging = None

    SL_X0  = 110
    SL_LEN = W - 220
    sliders = [
        {"key": "x", "label": "X", "min": -150, "max": 150, "y": H - 80},
        {"key": "y", "label": "Y", "min": -150, "max": 150, "y": H - 53},
        {"key": "z", "label": "Z", "min": -150, "max": 150, "y": H - 26},
    ]

    def c2s(v, mn, mx):
        return SL_X0 + int((v - mn) / (mx - mn) * SL_LEN)

    def s2c(px, mn, mx):
        return mn + max(0.0, min(1.0, (px - SL_X0) / SL_LEN)) * (mx - mn)

    running = True
    while running:
        clock.tick(FPS)

        cxy = (VP_XY[0] + VP_W // 2, VP_XY[1] + VP_H // 2 + 20)
        cxz = (VP_XZ[0] + VP_W // 2, VP_XZ[1] + VP_H // 2 + 20)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False

            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos
                if VP_XY[0] < mx < VP_XY[0] + VP_W and VP_XY[1] < my < VP_XY[1] + VP_H:
                    dragging_xy = True
                elif VP_XZ[0] < mx < VP_XZ[0] + VP_W and VP_XZ[1] < my < VP_XZ[1] + VP_H:
                    dragging_xz = True
                else:
                    for i, sl in enumerate(sliders):
                        if abs(my - sl["y"]) < 12:
                            slider_dragging = i
                            break

            if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                dragging_xy = dragging_xz = False
                slider_dragging = None

            if event.type == pygame.MOUSEMOTION:
                mx, my = event.pos
                if dragging_xy:
                    wx, wy = vp_to_world(mx, my, *cxy)
                    target_x = max(-150, min(150, wx))
                    target_y = max(-150, min(150, wy))
                elif dragging_xz:
                    wx, wz = vp_to_world(mx, my, *cxz)
                    target_x = max(-150, min(150, wx))
                    target_z = max(-150, min(150, wz))
                elif slider_dragging is not None:
                    sl = sliders[slider_dragging]
                    v  = s2c(mx, sl["min"], sl["max"])
                    if sl["key"] == "x": target_x = v
                    elif sl["key"] == "y": target_y = v
                    elif sl["key"] == "z": target_z = v

        # ── Draw ────────────────────────────────────────────────────────────────
        screen.fill(BG)
        for gx in range(0, W, 30):
            for gy in range(0, H, 30):
                pygame.draw.circle(screen, GRID, (gx, gy), 1)

        # Title bar
        pygame.draw.rect(screen, PANEL_BG, (0, 0, W, 105))
        pygame.draw.line(screen, ACCENT, (0, 105), (W, 105), 1)
        screen.blit(font_big.render("ROBOTIC ARM  //  KINEMATIC VISUALISER  v2", True, ACCENT), (30, 16))
        screen.blit(font_label.render(
            f"L1={lower_arm_length}mm   L2={upper_arm_length}mm   REACH 36–164mm   "
            f"DIV/ZERO GUARD ACTIVE", True, DIM), (30, 50))
        # Typo notice
        warn = font_warn.render(
            "⚠  source typo: arm_z_x_kinematics had `y=1` instead of `z=1` — corrected here", True, (180, 140, 0))
        screen.blit(warn, (30, 68))
        screen.blit(font_label.render(
            "DRAG VIEWPORTS  OR  USE SLIDERS  |  ESC TO QUIT", True, DIM), (30, 86))

        # ── XY Viewport
        pygame.draw.rect(screen, PANEL_BG, (*VP_XY, VP_W, VP_H))
        draw_grid(screen, (*VP_XY, VP_W, VP_H), 20)
        draw_panel_border(screen, (*VP_XY, VP_W, VP_H), LINK1_COL)
        screen.blit(font_title.render("▸ X – Y  PLANE  (ELEVATION)", True, LINK1_COL),
                    (VP_XY[0] + 10, VP_XY[1] + 8))

        ok_xy, upper_deg, lower_deg = draw_arm_xy(
            screen, *VP_XY, VP_W, VP_H, target_x, target_y, font_label)

        tpx, tpy = world_to_vp(target_x, target_y, *cxy)
        ch_col = ACCENT if ok_xy else RED
        pygame.draw.line(screen, ch_col, (tpx - 9, tpy), (tpx + 9, tpy), 1)
        pygame.draw.line(screen, ch_col, (tpx, tpy - 9), (tpx, tpy + 9), 1)

        # ── XZ Viewport
        pygame.draw.rect(screen, PANEL_BG, (*VP_XZ, VP_W, VP_H))
        draw_grid(screen, (*VP_XZ, VP_W, VP_H), 20)
        draw_panel_border(screen, (*VP_XZ, VP_W, VP_H), ACCENT2)
        screen.blit(font_title.render("▸ X – Z  PLANE  (BASE ROTATION)", True, ACCENT2),
                    (VP_XZ[0] + 10, VP_XZ[1] + 8))

        ok_xz, base_deg = draw_arm_xz(
            screen, *VP_XZ, VP_W, VP_H, target_x, target_z, font_label)

        tpx2, tpz2 = world_to_vp(target_x, target_z, *cxz)
        ch_col2 = ACCENT if ok_xz else RED
        pygame.draw.line(screen, ch_col2, (tpx2 - 9, tpz2), (tpx2 + 9, tpz2), 1)
        pygame.draw.line(screen, ch_col2, (tpx2, tpz2 - 9), (tpx2, tpz2 + 9), 1)

        # ── Info panel
        ip_x = VP_XY[0] + VP_W + 10
        ip_y = VP_XY[1]
        ip_w = VP_XZ[0] - ip_x - 10
        ip_h = VP_H
        pygame.draw.rect(screen, PANEL_BG, (ip_x, ip_y, ip_w, ip_h))
        draw_panel_border(screen, (ip_x, ip_y, ip_w, ip_h), DIM, 80)

        def info_row(label, val, y_off, color=WHITE):
            screen.blit(font_label.render(label, True, DIM),  (ip_x + 8, ip_y + y_off))
            screen.blit(font_val.render(val,     True, color), (ip_x + 8, ip_y + y_off + 14))

        screen.blit(font_title.render("POSITION", True, ACCENT), (ip_x + 8, ip_y + 8))
        info_row("X (mm)", f"{target_x:+.1f}", 28)
        info_row("Y (mm)", f"{target_y:+.1f}", 62)
        info_row("Z (mm)", f"{target_z:+.1f}", 96)

        dist = sqrt(target_x**2 + target_y**2 + target_z**2)
        info_row("DIST",   f"{dist:.1f}",       130, ACCENT2)

        screen.blit(font_title.render("ANGLES", True, ACCENT), (ip_x + 8, ip_y + 178))
        if ok_xy and upper_deg is not None:
            info_row("SHOULDER", f"{lower_deg:.1f}°",  198, LINK1_COL)
            info_row("ELBOW",    f"{upper_deg:.1f}°",  232, LINK2_COL)
        else:
            info_row("SHOULDER", "---", 198, RED)
            info_row("ELBOW",    "---", 232, RED)

        if ok_xz and base_deg is not None:
            info_row("BASE", f"{base_deg:.1f}°", 266, ACCENT2)
        else:
            info_row("BASE", "---", 266, RED)

        full = arm_full_kinematics(target_x, target_y, target_z)
        s_col = GREEN if full else RED
        s_txt = "REACHABLE" if full else "OUT OF RANGE"
        pygame.draw.rect(screen, PANEL_BG, (ip_x + 5, ip_y + 318, ip_w - 10, 32))
        draw_panel_border(screen, (ip_x + 5, ip_y + 318, ip_w - 10, 32), s_col)
        screen.blit(font_val.render(s_txt, True, s_col), (ip_x + 8, ip_y + 328))

        # guard indicator
        gx_txt = "DIV/ZERO: x=0→1" if abs(target_x) < 1 else "DIV/ZERO: OK"
        gy_txt = "DIV/ZERO: y=0→1" if abs(target_y) < 1 else ""
        gz_txt = "DIV/ZERO: z=0→1" if abs(target_z) < 1 else ""
        for i, gt in enumerate([gx_txt, gy_txt, gz_txt]):
            if gt:
                screen.blit(font_warn.render(gt, True, (180, 140, 0)),
                            (ip_x + 8, ip_y + 365 + i * 14))

        # ── Sliders
        sv   = {"x": target_x, "y": target_y, "z": target_z}
        scol = {"x": LINK1_COL, "y": ACCENT, "z": ACCENT2}
        for sl in sliders:
            val = sv[sl["key"]]
            col = scol[sl["key"]]
            sy  = sl["y"]
            pygame.draw.line(screen, GRID, (SL_X0, sy), (SL_X0 + SL_LEN, sy), 2)
            pygame.draw.line(screen, col,  (SL_X0, sy), (c2s(val, sl["min"], sl["max"]), sy), 2)
            kx = c2s(val, sl["min"], sl["max"])
            draw_glow_circle(screen, col, (kx, sy), 8, 60)
            pygame.draw.circle(screen, col, (kx, sy), 6)
            pygame.draw.circle(screen, BG,  (kx, sy), 3)
            screen.blit(font_val.render(sl["label"], True, col),   (SL_X0 - 22, sy - 7))
            screen.blit(font_label.render(f"{val:+.0f}", True, WHITE), (SL_X0 + SL_LEN + 10, sy - 7))

        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()