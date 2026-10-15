# AMR 중앙 관제 서버

중앙 서버 PC에서 실행한다.

```bash
cd /home/kim/Control_Server
.venv/bin/python scripts/serverctl.py start
```

브라우저에서 **http://100.88.249.77:8081/robots/amr-01/**을 연다.
로그인 이름은 **operator**, 비밀번호는 기존에 설정한 공용 토큰이다.
AMR 연결과 주행 조작은 [실운용 실행법](../docs/README.md#4-중앙-관제-사용-시)을 사용한다.

서버를 종료할 때는 아래 명령을 실행한다.

```bash
cd /home/kim/Control_Server
.venv/bin/python scripts/serverctl.py stop
```
