> **이력 문서** · 2026-10-07 정리 시점의 이전 안내입니다. 현재 실행은 [최신 운영 문서](../../../README.md), 검증은 [검증 문서](../../../VALIDATION.md)를 사용합니다.

# 2026-10-06 직접 이식 복귀

이번 변경은 `/home/acus03/amr_ws` 안에 적용했다. Tailscale·장치 서비스·인증 토큰은 변경하지 않았다.
원본 소스 백업 외에 소스·스크립트·문서·관제 사본·기존 빌드 결과를 함께 보관했다.

전체 복귀 백업: `backups/before_pc_update_20261006_231101/workspace_before_update.tar.gz`
체크섬: `backups/before_pc_update_20261006_231101/workspace_before_update.tar.gz.sha256`

## 복귀 순서

1. MANUAL 전환·목표 취소·바퀴 정지를 확인한다. 실행 중인 주행 스택, 웹 API, remote_client를
   각각의 터미널에서 Ctrl+C로 종료한다. 다른 버전이 동시에 실행되지 않게 한다.
2. 새 셸에서 다음을 수행한다. 현재 업데이트 결과도 별도 폴더로 옮겨 보존한다.
   아래 디렉터리에 작업 후 새 파일을 추가했다면 그 내용도 함께 이동된다.

```bash
cd /home/acus03/amr_ws
sha256sum -c backups/before_pc_update_20261006_231101/workspace_before_update.tar.gz.sha256
rollback_saved="backups/updated_workspace_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$rollback_saved"
for entry in src scripts docs tests Control_Server build install sim reports README.md RELEASE.json PORTING_JAZZY.md env.sh run_amr.sh build_scene.sh isaac-sim.sh python.sh stm32_main_code.c 실행법.md can0.service .gitignore maps; do
  if [ -e "$entry" ] || [ -L "$entry" ]; then mv -- "$entry" "$rollback_saved/"; fi
done
tar -xzpf backups/before_pc_update_20261006_231101/workspace_before_update.tar.gz -C /home/acus03/amr_ws
```

3. 새 터미널에서 `amr`을 실행하고 복원된 이전 실행법을 따른다. 업데이트한 호스트 중앙 서버도
   `Control_Server/UPDATE_20261006.md`에 따라 이전 버전으로 복귀한다.

백업 복원은 이식 시 추가된 패키지와 빌드 결과가 남지 않도록 디렉터리를 옮긴 뒤 수행한다.
`bags`, 기존 로그, `backups`, 진단 결과는 이동하지 않는다. 의존성으로 추가 설치한
`ros-jazzy-octomap`은 이전 버전에 영향을 주지 않아 남겨둔다.
