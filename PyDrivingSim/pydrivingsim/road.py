import pygame
from pydrivingsim.object import VirtualObject
from pydrivingsim.world import World

_GRASS    = ( 60, 140,  40)
_SHOULDER = ( 80, 110,  50)
_ASPHALT  = ( 55,  55,  55)
_LINE     = (220, 220, 220)
_STRIPE   = (240, 240, 240)
_STOPLINE = (255, 255, 255)

_ROAD_TOP_Y  =  2.0   # world y of left road edge
_ROAD_BOT_Y  = -2.0   # world y of right road edge
_CENTER_Y    =  0.0   # dashed centre-line world y
_SHOULDER_W  =  1.5   # shoulder width beyond each edge (m)

_DASH_LEN    =  3.0   # dash length (m)
_DASH_GAP    =  2.0   # gap between dashes (m)
_CROSS_HW    =  2.5   # half-width of zebra crossing in x (total 5 m)
_STRIPE_W    =  0.5   # one zebra stripe width (m)
_STOPLINE_W  =  0.3   # stop-line thickness (m)


class Road(VirtualObject):
    _dt = 0.1

    def __init__(self):
        super().__init__(self._dt)

    # ── coordinate helpers ──────────────────────────────────────────────
    def _sx(self, wx):
        sf = World().scaling_factor
        return (wx - World().get_world_pos()[0]) * sf + World().screen_world_center[0]

    def _sy(self, wy):
        sf = World().scaling_factor
        return (World().get_world_pos()[1] - wy) * sf + World().screen_world_center[1]

    # ── rendering ───────────────────────────────────────────────────────
    def render(self):
        from pydrivingsim.trafficlight import TrafficLight

        screen = World().screen
        sf     = World().scaling_factor
        sw     = screen.get_width()

        # 1. grass background fills the whole screen
        screen.fill(_GRASS)

        # 2. shoulder band (slightly wider than road, different tone)
        sh_top = int(self._sy(_ROAD_TOP_Y + _SHOULDER_W))
        sh_bot = int(self._sy(_ROAD_BOT_Y - _SHOULDER_W))
        pygame.draw.rect(screen, _SHOULDER, (0, sh_top, sw, sh_bot - sh_top))

        # 3. road surface
        road_top = int(self._sy(_ROAD_TOP_Y))
        road_bot = int(self._sy(_ROAD_BOT_Y))
        road_h   = road_bot - road_top
        pygame.draw.rect(screen, _ASPHALT, (0, road_top, sw, road_h))

        # 4. zebra crossings AFTER each traffic light; stop line AT the light
        crossing_xs = [obj.pos[0] for obj in World().obj_list if type(obj) is TrafficLight]
        stripe_px   = max(2, int(_STRIPE_W * sf))
        for cx in crossing_xs:
            # crossing starts at the traffic light x and extends forward
            lx = int(self._sx(cx))
            rx = int(self._sx(cx + _CROSS_HW * 2))
            if rx < 0 or lx > sw:
                continue
            # alternate white / asphalt stripes
            x    = lx
            fill = True
            while x < rx:
                ex = min(x + stripe_px, rx)
                if fill:
                    pygame.draw.rect(screen, _STRIPE, (x, road_top, ex - x, road_h))
                x    = ex
                fill = not fill
            # stop line at the traffic light position (where car stops)
            sl_x = int(self._sx(cx))
            sl_w = max(3, int(_STOPLINE_W * sf))
            pygame.draw.rect(screen, _STOPLINE, (sl_x - sl_w, road_top, sl_w, road_h))

        # 5. solid edge lines
        lw = max(2, int(0.08 * sf))
        pygame.draw.line(screen, _LINE, (0, road_top), (sw, road_top), lw)
        pygame.draw.line(screen, _LINE, (0, road_bot), (sw, road_bot), lw)

        # 6. dashed centre line
        dash_px   = int(_DASH_LEN * sf)
        gap_px    = int(_DASH_GAP * sf)
        period    = dash_px + gap_px
        centre_sy = int(self._sy(_CENTER_Y))
        dash_lw   = max(1, int(0.05 * sf))
        # phase offset keeps dashes stationary in world space as camera moves
        world_x_left = World().get_world_pos()[0] - World().screen_world_center[0] / sf
        phase = int(world_x_left * sf) % period
        xp = -phase
        while xp < sw:
            x0 = max(0, xp)
            x1 = min(sw, xp + dash_px)
            if x0 < x1:
                pygame.draw.line(screen, _LINE, (x0, centre_sy), (x1, centre_sy), dash_lw)
            xp += period
