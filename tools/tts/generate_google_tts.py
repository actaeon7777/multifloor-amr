#!/usr/bin/env python3
"""Generate the robot's announcement WAV files with Google Cloud Text-to-Speech.

Run on any PC with internet access (not needed on the Jetson):

    export GOOGLE_API_KEY=...        # Cloud TTS API key
    python3 tools/tts/generate_google_tts.py
    python3 tools/tts/generate_google_tts.py --list   # print file,text CSV only

Fixed phrases follow SPS-B-KOLA-0001-7700:2026 Annex C table C.1 where it has one.
Floor and room phrases are generated per floor/room from floors.yaml so the robot
never has to splice audio. Existing files are kept unless --force is given.
"""
import argparse
import base64
import csv
import json
import os
import sys
import urllib.request
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
FLOORS_YAML = ROOT / 'src/amr_system/amr_stack/config/floors.yaml'
API = 'https://texttospeech.googleapis.com/v1/text:synthesize'

FIXED = {
    'call': '지금 엘리베이터를 호출했습니다',
    'waiting': '엘리베이터 탑승을 대기 중입니다',
    'board_start': '승차를 시작합니다. 안전에 유의해 주세요',
    'board_space': '로봇이 탑승합니다. 조금만 비켜주세요',
    'board_cancel': '승차를 취소합니다. 안전에 유의해 주세요',
    'board_done': '승차가 완료되었습니다',
    'alight_start': '하차를 시작합니다. 안전에 유의해 주세요',
    'alight_cancel': '하차를 취소합니다. 안전에 유의해 주세요',
    'alight_done': '하차가 완료되었습니다',
    'elevator_error': '엘리베이터에서 오류가 발생했습니다. 지금 하차하겠습니다',
    'robot_error': '오류가 발생했습니다. 관리자를 호출해주세요',
    'obstacle': '죄송합니다. 잠시만 비켜주세요',
    'shock': '충격이 감지되었습니다. 위험할 수 있으니 조심해 주세요',
    'estop': '비상 정지되었습니다',
    'delivery_start': '배송을 시작합니다',
    'delivery_cancel': '배송을 취소합니다',
    'received': '수령이 완료되었습니다. 감사합니다',
}


def floor_label(level):
    """Building floor label (4.1.1 n): 4 -> '4층', -1 -> '지하 1층'."""
    return f'{level}층' if level > 0 else f'지하 {-level}층'


def phrases(floors_yaml):
    """(file stem, text) for every announcement, floor and room phrase."""
    config = yaml.safe_load(Path(floors_yaml).read_text(encoding='utf-8'))
    items = list(FIXED.items())
    for name, floor in config['floors'].items():
        items.append((f'arrival_{name}', f'{floor_label(floor["level"])}에서 하차하겠습니다. 잠시만 비켜주세요'))
        # "413-앞문"/"413-뒷문" are two doors of one room; announce the room once.
        for room in dict.fromkeys(str(r).split('-')[0] for r in floor.get('rooms', {})):
            items.append((f'room_{room}', f'{room}호에 도착했습니다. 물품을 수령해 주세요'))
    return items


def synthesize(text, key, voice, rate):
    body = json.dumps({
        'input': {'text': text},
        'voice': {'languageCode': 'ko-KR', 'name': voice},
        # LINEAR16 responses already carry a WAV header.
        'audioConfig': {'audioEncoding': 'LINEAR16', 'sampleRateHertz': 24000,
                        'speakingRate': rate},
    }).encode()
    request = urllib.request.Request(f'{API}?key={key}', data=body,
                                     headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(request, timeout=30) as response:
        return base64.b64decode(json.load(response)['audioContent'])


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--floors', default=FLOORS_YAML)
    parser.add_argument('--out', default=Path(__file__).resolve().parent / 'wav', type=Path)
    parser.add_argument('--voice', default='ko-KR-Neural2-A')
    parser.add_argument('--rate', default=1.0, type=float)
    parser.add_argument('--list', action='store_true', help='print file,text CSV and exit')
    parser.add_argument('--force', action='store_true', help='overwrite existing files')
    args = parser.parse_args()

    items = phrases(args.floors)
    if args.list:
        writer = csv.writer(sys.stdout)
        writer.writerow(['file', 'text'])
        writer.writerows((f'{stem}.wav', text) for stem, text in items)
        return
    key = os.environ.get('GOOGLE_API_KEY')
    if not key:
        sys.exit('GOOGLE_API_KEY is not set')
    args.out.mkdir(parents=True, exist_ok=True)
    for stem, text in items:
        path = args.out / f'{stem}.wav'
        if path.exists() and not args.force:
            continue
        path.write_bytes(synthesize(text, key, args.voice, args.rate))
        print(f'{path.name}: {text}')


if __name__ == '__main__':
    main()
