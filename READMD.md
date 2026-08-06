# 생성 (프로젝트 디렉터리에서, 1회)
python3 -m venv .venv

# 활성화 — 프롬프트에 (.venv)가 붙습니다
source .venv/bin/activate        # 리눅스/macOS
# .venv\Scripts\activate        # Windows

pip install ipympl scipy matplotlib pytest jupyterlab ipykernel numpy matplotlib     # 이제 이 방에만 설치됩니다
pip freeze > requirements.txt    # 설치 목록을 기록 (팀 공유용)

deactivate                       # 방에서 나오기





# jupyter notebook
python -m ipykernel install --user --name pose_lab --display-name "Python (pose_lab)"

# 실행
jupyter lab




# test 
python -m pytest test/test_module.py

