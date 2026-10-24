"""Real Chromium WebDriver check, using the installed snap and no Selenium dependency."""
import base64
import json
import math
from pathlib import Path
import socket
import subprocess
import sys
import time
from urllib.request import Request, urlopen
from urllib.error import HTTPError

from amr_web_ui.server import SharedState, load_preview

ROOT = Path(__file__).resolve().parents[4]
ARTIFACTS = ROOT/'diagnostics/concept_a_20261007/browser'
ARTIFACTS.mkdir(parents=True, exist_ok=True)


def port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def wait(predicate, seconds=20):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        try:
            value = predicate()
            if value:
                return value
        except Exception:
            pass
        time.sleep(.15)
    raise AssertionError('Browser check timed out')


def main():
    http_port, driver_port = port(), port()
    driver_log = (ARTIFACTS/'chromedriver.log').open('w')
    server_log = (ARTIFACTS/'preview.log').open('w')
    preview = subprocess.Popen([sys.executable, '-m', 'amr_web_ui.server', '--preview-map',
        str(ROOT/'src/amr_system/amr_stack/maps/G_4_floor/4_floor_0522.yaml'), '--host', '127.0.0.1', '--port', str(http_port)],
        stdout=server_log, stderr=subprocess.STDOUT)
    driver = subprocess.Popen(['/snap/bin/chromium.chromedriver', f'--port={driver_port}'],
                              stdout=driver_log, stderr=subprocess.STDOUT)
    base = f'http://127.0.0.1:{driver_port}'
    root = f'http://127.0.0.1:{http_port}'
    session = None

    def api(method, path, payload=None):
        req = Request(base+path, data=json.dumps(payload).encode() if payload is not None else None,
                      method=method, headers={'Content-Type':'application/json'})
        try:
            result = json.load(urlopen(req, timeout=30))['value']
        except HTTPError as error:
            raise AssertionError(error.read().decode()) from error
        if isinstance(result, dict) and result.get('error'):
            raise AssertionError(result)
        return result

    def script(text):
        return api('POST', f'/session/{session}/execute/sync', dict(script=text, args=[]))

    def state():
        return json.load(urlopen(root+'/api/state', timeout=2))

    def actions(x, y, end=None):
        steps = [dict(type='pointerMove', duration=0, x=round(x), y=round(y), origin='viewport'),
                 dict(type='pointerDown', button=0)]
        if end:
            steps.append(dict(type='pointerMove', duration=300, x=round(end[0]), y=round(end[1]), origin='viewport'))
        steps.append(dict(type='pointerUp', button=0))
        api('POST', f'/session/{session}/actions', dict(actions=[
            dict(type='pointer', id='finger', parameters=dict(pointerType='touch'), actions=steps)]))

    try:
        wait(lambda: urlopen(root+'/api/state', timeout=1).status == 200)
        wait(lambda: api('GET', '/status'))
        result = api('POST', '/session', dict(capabilities=dict(alwaysMatch={
            'browserName':'chrome', 'goog:chromeOptions':dict(binary='/snap/chromium/current/usr/lib/chromium-browser/chrome', args=[
                '--headless=new', '--no-sandbox', '--disable-dev-shm-usage', '--window-size=1440,1000']),
            'goog:loggingPrefs':{'browser':'ALL'}})))
        session = result['sessionId']
        api('POST', f'/session/{session}/url', dict(url=root))
        wait(lambda: script("return !document.getElementById('empty').hidden ? false : document.getElementById('map-meta').textContent.includes('1014')"))
        time.sleep(.5)
        shared = SharedState(preview=True)
        load_preview(shared, ROOT/'src/amr_system/amr_stack/maps/G_4_floor/4_floor_0522.yaml')
        grid = shared.grids['map']
        w, h = grid.info['width'], grid.info['height']
        _, i, j = min((abs(i-w/2)+abs(j-h/2), i, j) for j in range(h) for i in range(w)
                      if grid.data[j*w+i] == 0)
        target = dict(x=grid.info['origin'][0]+(i+.5)*grid.info['resolution'],
                      y=grid.info['origin'][1]+(j+.5)*grid.info['resolution'])
        rect = script("const r=document.getElementById('map').getBoundingClientRect();return {x:r.x,y:r.y,width:r.width,height:r.height}")
        scale = min((rect['width']-64)/(w*.05), (rect['height']-100)/(h*.05))
        center = dict(x=grid.info['origin'][0]+w*.05/2,y=grid.info['origin'][1]+h*.05/2)
        x=rect['x']+rect['width']/2+(target['x']-center['x'])*scale
        y=rect['y']+rect['height']/2-(target['y']-center['y'])*scale
        actions(x,y)
        wait(lambda: state()['goal'])
        goal=state()['goal']
        assert goal['status']=='PREVIEW'
        assert abs(goal['x']-target['x']) < 1/scale
        assert abs(goal['y']-target['y']) < 1/scale
        wait(lambda: script("return document.getElementById('goal-label').textContent==='미리보기'"))
        actions(x,y,end=(x+35,y-35))
        wait(lambda: state()['goal']['request_id'] != goal['request_id'])
        dragged=state()['goal']
        assert abs(dragged['yaw']-math.pi/4) < .04
        script("document.querySelector('[data-mode=pan]').click()")
        actions(x,y,end=(x+35,y+20))
        assert state()['goal']['request_id']==dragged['request_id']
        script("document.querySelector('[data-mode=goal]').click()")
        cx,cy=rect['x']+rect['width']/2,rect['y']+rect['height']/2
        api('POST', f'/session/{session}/actions', dict(actions=[
            dict(type='pointer',id='pinch_a',parameters=dict(pointerType='touch'),actions=[
                dict(type='pointerMove',duration=0,x=round(cx-30),y=round(cy),origin='viewport'),
                dict(type='pointerDown',button=0),
                dict(type='pointerMove',duration=200,x=round(cx-50),y=round(cy),origin='viewport'),
                dict(type='pointerUp',button=0)]),
            dict(type='pointer',id='pinch_b',parameters=dict(pointerType='touch'),actions=[
                dict(type='pointerMove',duration=0,x=round(cx+30),y=round(cy),origin='viewport'),
                dict(type='pointerDown',button=0),
                dict(type='pointerMove',duration=200,x=round(cx+50),y=round(cy),origin='viewport'),
                dict(type='pointerUp',button=0)])]))
        assert state()['goal']['request_id']==dragged['request_id']
        script("document.getElementById('fit').click()")
        time.sleep(.4)
        png=api('GET',f'/session/{session}/screenshot')
        (ARTIFACTS/'desktop.png').write_bytes(base64.b64decode(png))
        api('POST',f'/session/{session}/goog/cdp/execute', dict(cmd='Emulation.setDeviceMetricsOverride',
            params=dict(width=390,height=844,deviceScaleFactor=1,mobile=True)))
        time.sleep(.4)
        assert script('return document.documentElement.scrollWidth <= window.innerWidth+1')
        script("document.getElementById('fit').click()")
        rect = script("const r=document.getElementById('map').getBoundingClientRect();return {x:r.x,y:r.y,width:r.width,height:r.height}")
        scale=min((rect['width']-64)/(w*.05),(rect['height']-100)/(h*.05))
        x=rect['x']+rect['width']/2+(target['x']-center['x'])*scale
        y=rect['y']+rect['height']/2-(target['y']-center['y'])*scale
        actions(x,y)
        wait(lambda: state()['goal']['request_id'] != dragged['request_id'])
        mobile_goal=state()['goal']
        assert abs(mobile_goal['x']-target['x']) < 1/scale
        assert abs(mobile_goal['y']-target['y']) < 1/scale
        time.sleep(.3)
        png=api('GET',f'/session/{session}/screenshot')
        (ARTIFACTS/'mobile.png').write_bytes(base64.b64decode(png))
        script("document.getElementById('cancel').click()")
        wait(lambda: state()['goal'] is None)

        # The new shell must preserve the map and its command behavior across views.
        script("document.getElementById('menu-toggle').click()")
        assert script("return document.getElementById('main-menu').classList.contains('open') && document.getElementById('menu-toggle').getAttribute('aria-expanded')==='true'")
        script("document.querySelector('[data-view=vehicle-status]').click()")
        wait(lambda: script("return !document.getElementById('vehicle-view').hidden && document.getElementById('main-view').hidden"))
        assert script("return !document.getElementById('main-menu').classList.contains('open') && document.getElementById('menu-backdrop').hidden")
        assert script("return document.getElementById('health-title').textContent.includes('미리보기') && document.getElementById('side-battery-percent').textContent==='—'")
        # Third view: floor transit (steps, door, beacons) renders without a mission in preview.
        script("document.querySelector('[data-view=floor-transit]').click()")
        wait(lambda: script("return !document.getElementById('transit-view').hidden && document.getElementById('vehicle-view').hidden"))
        wait(lambda: script("return document.querySelectorAll('#transit-steps .transit-step').length===11 && document.querySelectorAll('#transit-beacons .beacon-card').length>0"))
        assert script("return document.getElementById('transit-route').textContent.includes('없습니다') && document.getElementById('transit-door-state').textContent==='미수신'")
        script("document.querySelector('[data-view=vehicle-status]').click()")
        wait(lambda: script("return !document.getElementById('vehicle-view').hidden && document.getElementById('transit-view').hidden"))
        for viewport in [(1440,1080),(1366,900),(900,900),(768,900),(390,844)]:
            api('POST',f'/session/{session}/goog/cdp/execute', dict(cmd='Emulation.setDeviceMetricsOverride',
                params=dict(width=viewport[0],height=viewport[1],deviceScaleFactor=1,mobile=viewport[0]<641)))
            assert script('return document.documentElement.scrollWidth <= window.innerWidth+1'), viewport
        png=api('POST',f'/session/{session}/goog/cdp/execute', dict(cmd='Page.captureScreenshot',
            params=dict(captureBeyondViewport=True)))['data']
        (ARTIFACTS/'vehicle-mobile-full.png').write_bytes(base64.b64decode(png))

        # Exercise real rendering with fresh, stale, and disconnected fixtures.
        checks = script("""
          const sensor={age:0.01,hz:20,linear:0.32};
          const fixture={preview:false,mode:'AUTO',navigation:'RUNNING',action_ready:true,
            localization_reset_pending:false,tf_error:null,robot:{age:0,x:1,y:2,yaw:0},
            localization:{age:0,ready:true,state:'READY',detail:'検証 <img src=x onerror=alert(1)>'},
            time_health:{age:0,timeout:0.6,healthy:true},
            goal:{request_id:'browser_test',status:'ACCEPTED',distance_remaining:3,feedback_age:0},
            telemetry:{scan:sensor,wheel:sensor,imu:sensor,ekf:sensor,command:sensor,
              vehicle:{age:0,mode:'AUTO',navigation:'RUNNING'},pose:{age:0,x:1,y:2,yaw:0},
              localization:{age:0,ready:true,state:'READY',detail:'検証 <img src=x onerror=alert(1)>'},
              time:{age:0,timeout:0.6,healthy:true},
              joints:{age:0,left_rpm:60,right_rpm:30},
              diagnostics:{age:0,detail:'diagnostic test',counters:{invalid:'7',stale_frames:'8',wheel_rebases:'1'}},
              journey:{request_id:'browser_test',travelled:2,complete:true}}};
          AMRDashboard.update(fixture,true);
          const fresh=document.getElementById('drive-progress').textContent==='40' &&
            document.getElementById('travelled').textContent==='2.0' &&
            document.getElementById('motor-left-rpm').textContent==='60.0 rpm' &&
            document.getElementById('health-title').textContent==='자동주행 조건 충족' &&
            document.getElementById('counter-invalid').textContent==='7';
          fixture.telemetry.motor_left_power={age:0,timeout:2,voltage:26.12,current:0.82};
          fixture.telemetry.motor_right_power={age:0,timeout:2,voltage:26.16,current:0.84};
          fixture.telemetry.motor_left_heartbeat={age:0,timeout:2,hz:10,axis_state:8,error_code:0};
          AMRDashboard.update(fixture,true);
          const power=document.getElementById('side-battery-percent').textContent==='30' &&
            document.getElementById('health-battery-percent').textContent==='30' &&
            document.getElementById('power-left-voltage').textContent==='26.12' &&
            document.getElementById('motor-left-error').textContent==='0x00000000';
          const relayed=structuredClone(fixture);
          let relayStable=true;
          for (const delay of [0.15,0.35,0.55,0.9,0.2,0.65,0.3,0.8,0.4]) {
            const snapshot=JSON.parse(JSON.stringify(relayed));
            snapshot.remote={age:delay,timeout:2};
            for (const value of Object.values(snapshot.telemetry)) {
              if (Number.isFinite(value?.age)) { value.source_age=value.age; value.age+=delay; }
            }
            AMRDashboard.update(snapshot,true);
            relayStable &&= document.getElementById('motor-left-rpm').textContent==='60.0 rpm' &&
              document.getElementById('sensor-wheel-hz').textContent==='20.0' &&
              document.getElementById('health-time').textContent==='유효' &&
              document.getElementById('health-battery-percent').textContent==='30';
          }
          relayed.remote={age:.6,timeout:2};
          relayed.telemetry.motor_left_power.fresh=false;
          AMRDashboard.update(relayed,true);
          const sourceExpired=document.getElementById('power-left-voltage').textContent==='—' &&
            document.getElementById('health-battery-percent').textContent==='—' &&
            document.getElementById('motor-left-rpm').textContent==='60.0 rpm';
          relayed.remote.age=2.1;
          AMRDashboard.update(relayed,true);
          const relayExpired=document.getElementById('motor-left-rpm').textContent==='—' &&
            document.getElementById('health-battery-percent').textContent==='—';
          fixture.telemetry.battery={age:0,timeout:2.5,soc:null,voltage:26.14,charge_state:0};
          AMRDashboard.update(fixture,true);
          const derived=document.getElementById('power-source').textContent==='CAN 전압' &&
            document.getElementById('side-battery-percent').textContent==='30' &&
            document.getElementById('side-battery-note').textContent==='전압 추정 · 26.14 V';
          fixture.telemetry.motor_left_power.age=3;
          fixture.telemetry.battery={age:3,timeout:2.5,fresh:false,voltage:null,soc:null};
          AMRDashboard.update(fixture,true);
          const partial=document.getElementById('side-battery-percent').textContent==='—' &&
            document.getElementById('power-left-voltage').textContent==='—' &&
            document.getElementById('power-right-voltage').textContent==='26.16';
          fixture.telemetry.motor_left_power.age=0;
          fixture.telemetry.battery={age:0,timeout:2,soc:78,voltage:26.5,charge_state:2,temperature:32.1};
          AMRDashboard.update(fixture,true);
          const bms=document.getElementById('side-battery-percent').textContent==='78' &&
            document.getElementById('health-battery-percent').textContent==='78' &&
            document.getElementById('side-battery-note').textContent==='BMS 잔량' &&
            document.getElementById('battery-charge-state').textContent==='충전 중';
          fixture.telemetry.wheel={...sensor,age:3};
          fixture.telemetry.joints={...fixture.telemetry.joints,age:3};
          fixture.telemetry.diagnostics={...fixture.telemetry.diagnostics,age:5};
          fixture.telemetry.time.age=1;
          fixture.telemetry.localization.ready=false;
          AMRDashboard.update(fixture,true);
          const stale=document.getElementById('drive-progress').textContent==='—' &&
            document.getElementById('motor-left-rpm').textContent==='—' &&
            document.getElementById('sensor-wheel-state').textContent==='지연' &&
            document.getElementById('counter-invalid').textContent==='—' &&
            document.getElementById('health-time').textContent==='미수신' &&
            !document.getElementById('health-detail').querySelector('img');
          AMRDashboard.update(fixture,false);
          const offline=document.getElementById('sensor-scan-state').textContent==='미수신' &&
            document.getElementById('side-battery-percent').textContent==='—' &&
            !document.getElementById('offline-banner').hidden;
          return {fresh,stale,offline,power,bms,derived,partial,relayStable,sourceExpired,relayExpired};
        """)
        assert all(checks.values()), checks
        assert state()['goal'] is None

        # Open health directly, then reveal the map: its first fit must use visible dimensions.
        api('POST', f'/session/{session}/url', dict(url=root+'/#vehicle-status'))
        api('POST', f'/session/{session}/refresh', {})
        wait(lambda: script("return document.getElementById('health-title').textContent.includes('미리보기')"))
        api('POST',f'/session/{session}/goog/cdp/execute', dict(cmd='Emulation.setDeviceMetricsOverride',
            params=dict(width=1440,height=1080,deviceScaleFactor=1,mobile=False)))
        time.sleep(.3)
        png=api('POST',f'/session/{session}/goog/cdp/execute', dict(cmd='Page.captureScreenshot',
            params=dict(captureBeyondViewport=True)))['data']
        (ARTIFACTS/'vehicle-desktop-full.png').write_bytes(base64.b64decode(png))
        script("document.querySelector('[data-view=main]').click()")
        wait(lambda: script("return !document.getElementById('main-view').hidden && document.getElementById('map').width>500"))
        time.sleep(.3)
        assert script("return parseFloat(document.getElementById('scale-line').style.width)>40")
        png=api('GET',f'/session/{session}/screenshot')
        (ARTIFACTS/'desktop-clarity.png').write_bytes(base64.b64decode(png))
        logs=api('POST',f'/session/{session}/log',dict(type='browser'))
        errors=[log for log in logs if log['level']=='SEVERE']
        assert not errors, errors
        report=dict(touch_goal=True,drag_heading=True,pan_does_not_send=True,pinch_does_not_send=True,
                    mobile_touch_goal=True,preview_cancel=True,mobile_no_horizontal_overflow=True,
                    mobile_menu=True,vehicle_view=True,vehicle_widths=[1440,1366,900,768,390],
                    vehicle_live_render=checks,direct_health_map_fit=True,
                    console_errors=errors,actual_map=[w,h])
        (ARTIFACTS/'browser_result.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report))
    finally:
        if session:
            try: api('DELETE', f'/session/{session}')
            except Exception: pass
        for process in (driver, preview):
            process.terminate()
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired: process.kill(); process.wait()
        driver_log.close(); server_log.close()


if __name__ == '__main__':
    main()
