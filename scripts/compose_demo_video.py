#!/usr/bin/env python3
"""Compose the multi-floor demo video from sim/scripts/demo_recorder.py frames.

  scripts/compose_demo_video.py reports/demo_X/raw reports/demo_X/events.jsonl reports/demo_X/demo.mp4

Main view full frame, the second view as an inset at the top right, captions from
the floor_demo.py events (leg, mission phase, floor, elapsed time) and labels
projected by the recorder. Real time: the frame rate is the recorded sim-time frame interval
(renders land on physics steps, e.g. 0.035 s = 28.6 fps).
System python3 (OpenCV with FFmpeg, Pillow, NumPy); H.264 when available.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

SIZE = (1920, 1080)
INSET_AT = (SIZE[0] - 640 - 36, 36)
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-{}.ttc'
KOREAN = 1  # face index of Noto Sans CJK KR in the collection
# Same wording as the web UI (src/amr_system/amr_web_ui/web/app.js missionLabels).
PHASES = {'NAV_TO_HALL': '엘리베이터 홀로 이동', 'CALL_ELEVATOR': '엘리베이터 호출', 'WAIT_DOOR_OPEN': '문 열림 대기',
          'BOARDING': '탑승 중', 'BACKING_OUT': '탑승 중단 · 홀로 복귀', 'IN_CAR_ALIGN': '엘리베이터 안 정렬',
          'SELECT_FLOOR': '층 버튼 선택', 'RIDING': '층 이동 중', 'SWITCH_MAP': '지도 전환', 'ALIGHTING': '하차 중',
          'RETURN_TO_CAR': '하차 중단 · 카 안 복귀', 'RELOCALIZE': '위치 재확인', 'NAV_TO_GOAL': '목적지로 이동',
          'SUCCEEDED': '도착', 'FAILED_SAFE': '안전 정지', 'CANCELED': '취소'}
MAIN_NAMES = {'chase': '추적 시점', 'hall': '엘리베이터 홀', 'section': '건물 단면 · 승강로'}
ACCENT = (40, 170, 255)
OUTLINE = {'stroke_width': 2, 'stroke_fill': (0, 0, 0)}


def font(weight, size):
    return ImageFont.truetype(FONT.format(weight), size, index=KOREAN)


def room_label(room):
    number, _, door = room.partition('-')
    return f'{number}호 {door}' if door else f'{number}호'


class Captions:
    """Caption state at a sim time, from the floor_demo.py events."""

    def __init__(self, events):
        self.events = sorted((e for e in events if e.get('sim_time') is not None), key=lambda e: e['sim_time'])
        self.stops = [e for e in self.events if e['event'] == 'goal']
        self.start = next((e['sim_time'] for e in self.events if e['event'] == 'ready'), 0.)

    def at(self, t):
        state = {'leg': None, 'phase': '시작 준비', 'done': {}, 'index': -1}
        for e in self.events:
            if e['sim_time'] > t:
                break
            if e['event'] == 'goal':
                state['index'] += 1
                start = '' if e['start_floor'] == e['floor'] else f"{e['start_floor']} → "
                state['leg'] = f"{start}{e['floor']} {room_label(e['room'])}"
                state['phase'] = '출발'
            elif e['event'] == 'phase' and e.get('phase'):
                state['phase'] = PHASES.get(e['phase'], e['phase'])
            elif e['event'] == 'arrived':
                state['done'][state['index']] = e['error_m']
                state['phase'] = f"{room_label(e['room'])} 도착 · 위치 오차 {e['error_m']:.2f} m"
            elif e['event'] in ('failed', 'timeout', 'rejected'):
                state['done'][state['index']] = None
                state['phase'] = '실패: ' + e.get('message', e['event'])
        return state


def pill(draw, xy, text, typeface, fill=(255, 255, 255), background=(0, 0, 0, 170), anchor='mm', pad=(14, 6)):
    box = draw.textbbox(xy, text, font=typeface, anchor=anchor)
    draw.rounded_rectangle((box[0] - pad[0], box[1] - pad[1], box[2] + pad[0], box[3] + pad[1]), 10,
                           fill=background)
    draw.text(xy, text, font=typeface, fill=fill, anchor=anchor)


def compose(raw, events, output, title):
    meta = [json.loads(line) for line in (raw / 'meta.jsonl').read_text().splitlines() if line.strip()]
    captions = Captions(events)
    stops = captions.stops
    fonts = {'title': font('Bold', 30), 'leg': font('Bold', 44), 'phase': font('Medium', 34),
             'small': font('Medium', 24), 'label': font('Bold', 30), 'chip': font('Medium', 26)}
    times = np.array([record['sim_time'] for record in meta])
    fps = 1 / float(np.median(np.diff(times))) if len(times) > 1 else 30.
    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*'avc1'), fps, SIZE)
    codec = 'H.264'
    if not writer.isOpened():
        writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*'mp4v'), fps, SIZE)
        codec = 'MPEG-4 Part 2'
    start = meta[0]['sim_time'] if meta else 0.
    for record in meta:
        index = record['frame']
        frame = Image.open(raw / 'main' / f'{index:06d}.jpg').convert('RGBA')
        inset = Image.open(raw / 'inset' / f'{index:06d}.jpg').convert('RGB')
        draw = ImageDraw.Draw(frame)
        # Inset with border and caption.
        x, y = INSET_AT
        draw.rectangle((x - 4, y - 4, x + inset.width + 3, y + inset.height + 3), fill=(255, 255, 255, 255))
        frame.paste(inset, (x, y))
        inset_name = f"{record['floor']} Top view" if record['inset'] == 'top' else '엘리베이터 내부'
        pill(draw, (x + 12, y + inset.height + 30), inset_name, fonts['small'], anchor='lm')
        # Projected labels in the main view.
        for label in record['labels']:
            text = room_label(label['text']) if label['kind'] == 'room' else label['text']
            pill(draw, (label['u'], label['v']), text, fonts['label'],
                 background=(*ACCENT, 220) if label['kind'] == 'room' else (0, 0, 0, 190))
        # Title and current camera.
        pill(draw, (36, 52), title, fonts['title'], anchor='lm', pad=(18, 10))
        pill(draw, (36, 112), MAIN_NAMES.get(record['main'], record['main']), fonts['small'], anchor='lm')
        # Caption: leg, phase, floor and time; outlined text instead of a shaded band keeps the view clear.
        t = record['sim_time']
        state = captions.at(t)
        if state['leg']:
            draw.text((48, SIZE[1] - 150), state['leg'], font=fonts['leg'], fill=(255, 255, 255), anchor='lm', **OUTLINE)
        draw.text((48, SIZE[1] - 92), state['phase'], font=fonts['phase'], fill=(*ACCENT, 255), anchor='lm', **OUTLINE)
        elapsed = max(0., t - start)
        draw.text((48, SIZE[1] - 42), f"현재 층 {record['floor']}  ·  경과 {int(elapsed // 60):02d}:{int(elapsed % 60):02d}"
                  "  ·  실시간 1배속", font=fonts['small'], fill=(235, 235, 235), anchor='lm', **OUTLINE)
        # Route chips at the bottom right.
        right = SIZE[0] - 40
        for number in range(len(stops) - 1, -1, -1):
            stop = stops[number]
            mark = ' ✓' if state['done'].get(number) is not None else ''
            text = f"{number + 1}. {stop['floor']} {room_label(stop['room'])}{mark}"
            box = draw.textbbox((right, SIZE[1] - 60), text, font=fonts['chip'], anchor='rm')
            current = number == state['index'] and number not in state['done']
            fill = (*ACCENT, 235) if current else (40, 40, 40, 200) if number not in state['done'] else (30, 120, 60, 220)
            draw.rounded_rectangle((box[0] - 16, box[1] - 10, box[2] + 16, box[3] + 10), 12, fill=fill)
            draw.text((right, SIZE[1] - 60), text, font=fonts['chip'], fill=(255, 255, 255), anchor='rm')
            right = box[0] - 44
        writer.write(cv2.cvtColor(np.asarray(frame.convert('RGB')), cv2.COLOR_RGB2BGR))
        if index % 900 == 0:
            print(f'{index} / {len(meta)} frames', flush=True)
    writer.release()
    duration = len(meta) / fps
    sim_span = meta[-1]['sim_time'] - meta[0]['sim_time'] if meta else 0.
    return {'output': str(output), 'codec': codec, 'fps': round(fps, 3), 'frames': len(meta),
            'duration_s': round(duration, 1),
            'sim_time_span_s': round(sim_span, 1)}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('raw', type=Path, help='AMR_DEMO_OUTPUT of the recording')
    parser.add_argument('events', type=Path, help='floor_demo.py --events file')
    parser.add_argument('output', type=Path)
    parser.add_argument('--title', default='G동 AMR 층간 자율주행 시연')
    args = parser.parse_args()
    events = [json.loads(line) for line in args.events.read_text().splitlines() if line.strip()]
    print(json.dumps(compose(args.raw, events, args.output, args.title), ensure_ascii=False))


if __name__ == '__main__':
    main()
