# AMR 실운용 실행

AMR 터미널 1에서 차량을 실행한다.

```bash
cd /home/acus03/amr_ws
./scripts/experiment.sh real
```

AMR 터미널 2에서 웹을 실행한다.

```bash
cd /home/acus03/amr_ws
source scripts/env.sh
export ROS_DOMAIN_ID=7
ros2 launch amr_web_ui web_ui.launch.py \
  use_sim_time:=false host:=127.0.0.1 port:=8080
```

AMR 브라우저에서 **http://127.0.0.1:8080**을 연다.
주행 조작, 원격 접속과 종료는 [실운용 실행법](docs/README.md)을 사용한다.
