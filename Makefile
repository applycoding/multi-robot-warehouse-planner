PYTHON := $(if $(wildcard .venv/bin/python),.venv/bin/python,python3)
.PHONY: install test e2e run docker-build docker-run screenshot

install:
	$(PYTHON) -m pip install -r requirements-dev.txt

test:
	$(PYTHON) -m pytest -m "not e2e"

e2e:
	$(PYTHON) -m playwright install --with-deps chromium
	$(PYTHON) -m pytest -m e2e

run:
	$(PYTHON) -m uvicorn app.main:app --host 0.0.0.0 --port 8000

docker-build:
	docker build -t mrwp .

docker-run:
	docker run --rm -p 8000:8000 mrwp

screenshot:
	$(PYTHON) scripts/screenshot.py
