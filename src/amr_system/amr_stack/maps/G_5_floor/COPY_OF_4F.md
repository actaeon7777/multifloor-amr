# 임시 지도 (4층 복사본)

이 폴더의 지도는 2026-10-13에 4층 지도를 그대로 복사한 **임시본**이다. 실측 지도가 아니다.

- 원본: `maps/G_4_floor/4_floor_0522.*`
- 층 설정: `config/floors.yaml` (`provisional_copy_of: "4F"`, 실차 `real_enabled: false`)
- 실측 지도로 교체할 때: 이 폴더의 PGM·YAML(·영역지정 PNG)을 바꾸고 `floors.yaml`의 경로와
  엘리베이터·호실 좌표를 갱신한 뒤 `provisional_copy_of`를 지운다. 코드는 수정하지 않는다.
