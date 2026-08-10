# 생성 (프로젝트 디렉터리에서, 1회)
```bash
python3 -m venv .venv
```

# 활성화 — 프롬프트에 (.venv)가 붙습니다
```bash
source .venv/bin/activate        # 리눅스/macOS
# .venv\Scripts\activate        # Windows
```

# setting and requirements
```bash
pip install ipympl scipy matplotlib pytest jupyterlab ipykernel numpy matplotlib     # 이제 이 방에만 설치됩니다
pip freeze > requirements.txt    # 설치 목록을 기록 (팀 공유용)
```

# development mode  module import
```shell
pip install -e .
```

# pip install   directory module
```shell
pip install .
```

# 1. 빌드 도구 설치
```shell
pip install build
```

# 2. 빌드 실행 (pyproject.toml이 있는 위치에서)
```shell
python -m build
```

# requirement.txt install
```shell
pip install -r requirements.txt
```

# pip uninstall
```shell
pip uninstall -r requirements.txt
pip uninstall package_name
```

# pip list
```shell
pip list
```

# 방에서 나오기
```bash
deactivate
```





# jupyter
- jupyter notebook
```bash
python -m ipykernel install --user --name pose_lab --display-name "Python (pose_lab)"
```

- jupyter 실행
```bash
jupyter lab
```


-----------

# test
```bash
python -m pytest test/test_module.py
```

# target execution
```bash
python main.py
```

