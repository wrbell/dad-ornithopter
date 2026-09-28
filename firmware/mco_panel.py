#!/usr/bin/env python3
"""
MCO-1D Ground Panel — Revision 3
Four buttons. Everything else is automatic or gesture-driven.

Hardware:  Official RPi 7" DSI touchscreen (800×480)
Install:   sudo apt install python3-pygame
Run:       python3 mco_panel.py              # fullscreen
           python3 mco_panel.py --windowed   # development

BUTTONS:
  PRE-FLIGHT   Hold 1.5s. Runs all three startup check phases,
               then arms ODrives and enables everything.
               Discovery runs from this point forward.

  POST-FLIGHT  Single press. Runs post-flight checks, then
               automatically sequences: cooldown (8 min) →
               aft stow → disarm. Walk away after pressing.

  STORM STOW   Single press. If post-flight is running:
               overrides aft stow with storm config.
               If pressed standalone: goes directly.
               Always ends with disarm + tie-down warning.

  EMERGENCY    No confirmation. Instant wing stop.
"""

import sys, socket, json, threading, time, math

try:
    import pygame, pygame.freetype
    HAS_PYGAME = True
except ImportError:
    pygame = None
    HAS_PYGAME = False

WINDOWED = '--windowed' in sys.argv
W, H     = 800, 480
FPS      = 30
GS_HOST  = 'localhost'
GS_PORT  = 5757

C = {
    'bg':     ( 14,  14,  20),
    'hdr':    ( 22,  22,  32),
    'border': ( 50,  50,  70),
    'text':   (225, 225, 235),
    'dim':    (110, 110, 135),
    'good':   ( 80, 210, 120),
    'warn':   (240, 185,  50),
    'danger': (220,  60,  60),
    'pre':    ( 35,  90,  55),
    'post':   ( 38,  62, 125),
    'storm':  (110,  70,   0),
    'emg':    (160,  20,  20),
    'active': ( 55, 130, 200),
    'hold':   ( 90,  70,   0),
}


class GSClient:
    def __init__(self):
        self.connected = False
        self.telem     = {}
        self._lock     = threading.Lock()
        self._alive    = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self._alive:
            try:
                s = socket.create_connection((GS_HOST, GS_PORT), 2.0)
                self.connected = True
                buf = b''
                while self._alive:
                    d = s.recv(4096)
                    if not d:
                        break
                    buf += d
                    while b'\n' in buf:
                        line, buf = buf.split(b'\n', 1)
                        try:
                            msg = json.loads(line)
                            if msg.get('type') in ('telemetry','status'):
                                with self._lock:
                                    self.telem.update(msg.get('data',{}))
                        except Exception:
                            pass
                s.close()
            except Exception:
                pass
            self.connected = False
            time.sleep(2.0)

    def send(self, cmd):
        try:
            s = socket.create_connection((GS_HOST, GS_PORT), 1.0)
            s.sendall((json.dumps({'type':'cmd','cmd':cmd})+'\n').encode())
            s.close()
            return True
        except Exception:
            return False

    def get(self):
        with self._lock:
            return dict(self.telem)


class Btn:
    R = 18
    def __init__(self, rect, label, sub, col, hold_s=0.0):
        self.rect   = pygame.Rect(rect)
        self.label  = label
        self.sub    = sub
        self.col    = col
        self.hold_s = hold_s
        self.down   = False
        self.down_t = None
        self.active = False

    def base_col(self):
        if self.down and self.hold_s > 0:
            return C['hold']
        return C['active'] if self.active else C[self.col]

    def draw(self, surf, fb, fs, now):
        bc = self.base_col()
        # Shadow
        pygame.draw.rect(surf,(5,5,8),self.rect.inflate(3,3).move(3,3),
                          border_radius=self.R)
        # Body
        pygame.draw.rect(surf, bc, self.rect, border_radius=self.R)
        # Border
        bcol = C['danger'] if self.col=='emg' else (
               C['warn'] if (self.down and self.hold_s>0) else C['border'])
        pygame.draw.rect(surf, bcol, self.rect, 2, border_radius=self.R)

        cx, cy = self.rect.centerx, self.rect.centery

        if self.down and self.hold_s > 0 and self.down_t:
            frac = min(1.0,(now-self.down_t)/self.hold_s)
            r    = self.rect.height//2 - 10
            ar   = pygame.Rect(cx-r,cy-r,2*r,2*r)
            if frac > 0.02:
                end_a = math.pi/2 - frac*2*math.pi
                pygame.draw.arc(surf, C['warn'], ar, end_a, math.pi/2, 6)
            text, sub = 'HOLD', 'to confirm'
        else:
            text, sub = self.label, self.sub

        lw,lh = fb.get_rect(text).size
        fb.render_to(surf,(cx-lw//2,cy-lh//2-7),text,C['text'])
        sw,sh = fs.get_rect(sub).size
        fs.render_to(surf,(cx-sw//2,cy+lh//2),sub,C['dim'])

    def touch_down(self, pos, now):
        if self.rect.collidepoint(pos):
            self.down   = True
            self.down_t = now
            return True
        return False

    def touch_up(self, pos, now):
        if not self.down:
            return False
        fired = False
        if self.rect.collidepoint(pos):
            if self.hold_s == 0:
                fired = True
            elif self.down_t and (now-self.down_t) >= self.hold_s:
                fired = True
        self.down   = False
        self.down_t = None
        return fired


class Panel:
    HOLD = 1.5

    def __init__(self):
        pygame.init()
        pygame.freetype.init()
        flags = pygame.FULLSCREEN|pygame.NOFRAME if not WINDOWED else 0
        self.screen = pygame.display.set_mode((W,H), flags)
        pygame.display.set_caption('MCO-1D')
        if not WINDOWED:
            pygame.mouse.set_visible(False)

        self.clock = pygame.time.Clock()
        self.gs    = GSClient()

        self.fb = pygame.freetype.SysFont('dejavusans', 22, bold=True)
        self.fm = pygame.freetype.SysFont('dejavusans', 13)
        self.fs = pygame.freetype.SysFont('dejavusans', 11)
        self.ft = pygame.freetype.SysFont('dejavusans',  9)
        self.fmono = pygame.freetype.SysFont('dejavusansmono', 10)

        M  = 14
        BH = H - 62 - M*3
        BW = (W - M*3) // 2
        y1 = 62 + M
        y2 = 62 + M + BH//2 + M

        self.btns = {
            'pre':   Btn((M,      y1, BW, BH//2-M),
                         'PRE-FLIGHT',  'Hold to arm & check', 'pre',  self.HOLD),
            'post':  Btn((M+BW+M, y1, BW, BH//2-M),
                         'POST-FLIGHT', 'Cooldown → stow → disarm', 'post', 0.0),
            'storm': Btn((M,      y2, BW, BH//2),
                         'STORM STOW',  'Oshkosh / outdoor park', 'storm', 0.0),
            'emg':   Btn((M+BW+M, y2, BW, BH//2),
                         'STOP',        'Emergency', 'emg',  0.0),
        }

        # Post-flight sequencer state
        self._pf_active  = False
        self._pf_stage   = None    # 'checks'|'cooldown'|'stowing'|'done'
        self._pf_start   = None
        self._cd_start   = None
        self.COOLDOWN_S  = 480.0

        # Pre-flight state
        self._pre_stage  = 'idle'  # idle|running|pass|fail

        self.msg   = ''
        self.msg_t = 0.0

    def flash(self, txt):
        self.msg   = txt
        self.msg_t = time.monotonic()

    def fire(self, bid, telem):
        if bid == 'pre':
            self._pre_stage = 'running'
            self.gs.send('preflight_start')
            self.flash('Pre-flight checks running…')

        elif bid == 'post':
            if self._pf_active:
                self.flash('Post-flight already running')
                return
            self._pf_active = True
            self._pf_stage  = 'checks'
            self._pf_start  = time.monotonic()
            self.gs.send('postflight_start')
            self.flash('Post-flight: checking…')

        elif bid == 'storm':
            if self._pf_active and self._pf_stage == 'cooldown':
                # Override cooldown → go to storm stow instead
                self._pf_stage = 'storm_override'
                self.gs.send('storm_stow')
                self.flash('Storm stow — DISARM + TIE-DOWNS')
            else:
                # Direct path
                self.gs.send('storm_stow')
                def _disarm():
                    time.sleep(22)
                    self.gs.send('disarm')
                threading.Thread(target=_disarm, daemon=True).start()
                self.flash('Storm stow — wings folding — disarm in 22s')

        elif bid == 'emg':
            self.gs.send('emergency_stop')
            self.flash('EMERGENCY STOP')

    def _pf_tick(self, telem, now):
        """Advance post-flight sequence."""
        if not self._pf_active:
            return

        if self._pf_stage == 'checks':
            elapsed = now - self._pf_start
            V_ms    = telem.get('V_ms', 0.0)
            if elapsed > 5.0 and V_ms < 0.5:
                self._pf_stage = 'cooldown'
                self._cd_start = now
                self.gs.send('ground_ops_cooldown')

        elif self._pf_stage == 'cooldown':
            cd_el = now - self._cd_start
            if cd_el >= self.COOLDOWN_S:
                self._pf_stage = 'stowing'
                self.gs.send('ground_ops_aft_stow')

        elif self._pf_stage == 'stowing':
            if not hasattr(self, '_stow_t'):
                self._stow_t = now
            if now - self._stow_t > 18.0:
                del self._stow_t
                self._pf_stage = 'done'
                self.gs.send('disarm')
                self.flash('Post-flight complete — safe to handle')
                self._pf_active = False

    def _cd_frac(self, now):
        if self._pf_stage == 'cooldown' and self._cd_start:
            return min(1.0, (now-self._cd_start)/self.COOLDOWN_S)
        return 0.0

    def _pf_status(self, now):
        if not self._pf_active:
            return ''
        if self._pf_stage == 'checks':
            return 'Post-flight checks…'
        if self._pf_stage == 'cooldown' and self._cd_start:
            rem = max(0, self.COOLDOWN_S-(now-self._cd_start))
            return f'Cooling {int(rem//60)}:{int(rem%60):02d}'
        if self._pf_stage == 'stowing':
            return 'Stowing wings…'
        if self._pf_stage == 'storm_override':
            return 'Storm stow…'
        return ''

    def draw_hdr(self, telem, now):
        pygame.draw.rect(self.screen, C['hdr'], (0,0,W,62))
        pygame.draw.line(self.screen, C['border'], (0,62),(W,62),1)

        # Connection dot
        pygame.draw.circle(self.screen,
            C['good'] if self.gs.connected else C['danger'], (14,31), 6)

        # Mode / status
        raw_mode = telem.get('ground_ops_mode',
                              telem.get('gnd_state','—'))
        pf_txt   = self._pf_status(now)
        pre_txt  = {'running':'Checking…','pass':'ARMED ✓',
                    'fail':'CHECK FAILED'}.get(self._pre_stage,'')
        display  = pf_txt or pre_txt or raw_mode.replace('_',' ').upper()
        col      = (C['danger'] if 'FAIL' in display else
                    C['good']  if '✓' in display else C['text'])
        mw,mh = self.fm.get_rect(display).size
        self.fm.render_to(self.screen,(26,31-mh//2),display,col)

        # Status bar items
        items = [
            ('SPD',  f"{telem.get('V_ms',0.0):.1f}m/s"),
            ('ALT',  f"{telem.get('h_agl_m',0.0)*100:.0f}cm"),
            ('BATT', f"{telem.get('bus_voltage_v',0.0):.1f}V"),
            ('TURB', f"{telem.get('turb_intensity',0.0):.2f}"),
            ('L9',   'ON'  if telem.get('l9_active')   else '—'),
            ('DISC', 'ON'  if telem.get('disc_active')  else '—'),
        ]
        x = 165
        for lbl, val in items:
            vc = C['text']
            if lbl=='BATT' and telem.get('bus_voltage_v',15)<13.5: vc=C['warn']
            if lbl=='DISC' and val=='ON': vc=C['good']
            if lbl=='TURB' and telem.get('turb_intensity',0)>0.3:  vc=C['warn']
            self.ft.render_to(self.screen,(x,10),lbl,C['dim'])
            self.fmono.render_to(self.screen,(x,24),val,vc)
            x += 96

        # Cooldown arc
        frac = self._cd_frac(now)
        if frac > 0:
            cx,cy,r = W-30,31,22
            pygame.draw.circle(self.screen,C['border'],(cx,cy),r,3)
            if frac > 0.02:
                ea = math.pi/2 - frac*2*math.pi
                ar = pygame.Rect(cx-r,cy-r,2*r,2*r)
                pygame.draw.arc(self.screen,C['good'],ar,ea,math.pi/2,5)
            pct = f'{int(frac*100)}%'
            pw,ph = self.ft.get_rect(pct).size
            self.ft.render_to(self.screen,(cx-pw//2,cy-ph//2),pct,C['good'])

    def draw_msg(self, now):
        if not self.msg:
            return
        age = now - self.msg_t
        if age > 3.0:
            self.msg = ''
            return
        a = max(0, int(255*(1-age/3.0)))
        surf = pygame.Surface((W-40,44), pygame.SRCALPHA)
        surf.fill((18,18,28,min(230,a+80)))
        pygame.draw.rect(surf,(*C['border'],a),(0,0,W-40,44),1,border_radius=8)
        mw,mh = self.fm.get_rect(self.msg).size
        self.fm.render_to(surf,((W-40)//2-mw//2,22-mh//2),self.msg,(*C['text'],a))
        self.screen.blit(surf,(20,H//2-22))

    def update_active(self, telem):
        mode = telem.get('ground_ops_mode','')
        self.btns['pre'].active   = (self._pre_stage == 'pass')
        self.btns['post'].active  = self._pf_active
        self.btns['storm'].active = (mode == 'storm_stow')
        self.btns['emg'].active   = False

    def draw_margin_alert(self, telem, now):
        """
        Draw margin warning bar at top of screen when margins are narrow.
        Colour: yellow=CAUTION, amber=WARNING, red=CRITICAL.
        The pilot reads it and decides what to do.
        """
        alert = telem.get('alert_level', 'NORMAL')
        if alert == 'NORMAL':
            return

        warnings = telem.get('warnings', [])
        if not warnings:
            return

        w_entry = warnings[0]
        msg     = w_entry.get('message', '')
        axis    = w_entry.get('axis', '')
        margin  = w_entry.get('margin', 1.0)
        conf    = w_entry.get('confidence', 'estimated')

        # Bar colour
        bar_col = (C['warn']   if alert == 'CAUTION'  else
                   (200,120,0) if alert == 'WARNING'   else
                   C['danger'])
        text_col = (0,0,0) if alert == 'CAUTION' else C['text']

        # Draw alert bar across full width
        bar_h = 28
        pygame.draw.rect(self.screen, bar_col, (0, 62, W, bar_h))

        # Alert text
        label = f"▲ {alert}: {msg}"
        if conf == 'estimated':
            label += '  [pre-TC estimates]'
        lw, lh = self.fs.get_rect(label).size
        self.fs.render_to(self.screen, (8, 62 + bar_h//2 - lh//2),
                           label, text_col)

        # Margin number on right
        m_txt = f"{margin:.3f}"
        mw, mh = self.fm.get_rect(m_txt).size
        self.fm.render_to(self.screen, (W-mw-8, 62 + bar_h//2 - mh//2),
                           m_txt, text_col)

    def run(self):
        running = True
        while running:
            now   = time.monotonic()
            telem = self.gs.get()
            self._pf_tick(telem, now)

            # Sync margin warnings from telemetry
            margins = telem.get('margins', {})
            telem['alert_level'] = margins.get('alert_level', 'NORMAL')
            telem['warnings']    = margins.get('warnings', [])

            # S1 motor health alerts — highest level wins
            motor_alert = telem.get('motor_alert_level', 'NORMAL')
            motor_alerts = telem.get('motor_alerts', [])
            if motor_alert != 'NORMAL':
                # Motor health overrides aero margin if more severe
                level_order = {'NORMAL': 0, 'CAUTION': 1,
                               'WARNING': 2, 'CRITICAL': 3}
                if level_order.get(motor_alert, 0) >= \
                   level_order.get(telem['alert_level'], 0):
                    telem['alert_level'] = motor_alert
                    # Prepend motor alerts to warnings list
                    motor_warn_items = [
                        {'message': a['msg'],
                         'axis':    a.get('wing',''),
                         'margin':  0.0,
                         'confidence': 'measured'}
                        for a in motor_alerts
                    ]
                    telem['warnings'] = (motor_warn_items +
                                         telem.get('warnings', []))

            # Sync pre-flight status from telemetry
            pf_status = telem.get('preflight_status','')
            if pf_status == 'pass' and self._pre_stage == 'running':
                self._pre_stage = 'pass'
                self.flash('ALL CHECKS PASSED — Armed')
            elif pf_status == 'fail' and self._pre_stage == 'running':
                self._pre_stage = 'fail'
                self.flash('PRE-FLIGHT FAILED — see ground station')

            self.update_active(telem)

            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running = False
                elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                    running = False
                elif ev.type in (pygame.MOUSEBUTTONDOWN, pygame.FINGERDOWN):
                    pos = (int(ev.x*W),int(ev.y*H)) \
                          if ev.type==pygame.FINGERDOWN else ev.pos
                    for b in self.btns.values():
                        b.touch_down(pos, now)
                elif ev.type in (pygame.MOUSEBUTTONUP, pygame.FINGERUP):
                    pos = (int(ev.x*W),int(ev.y*H)) \
                          if ev.type==pygame.FINGERUP else ev.pos
                    for bid, b in self.btns.items():
                        if b.touch_up(pos, now):
                            self.fire(bid, telem)

            self.screen.fill(C['bg'])
            self.draw_hdr(telem, now)
            self.draw_margin_alert(telem, now)
            for b in self.btns.values():
                b.draw(self.screen, self.fb, self.fs, now)
            self.draw_msg(now)
            pygame.display.flip()
            self.clock.tick(FPS)

        pygame.quit()


if __name__ == '__main__':
    if not HAS_PYGAME:
        print("ERROR: pygame not installed. Run: sudo apt install python3-pygame")
        sys.exit(1)
    Panel().run()
