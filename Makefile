MAP ?= config.txt
PYTHON ?= python3

.PHONY: all install run debug test clean lint lint-strict

all:
	$(PYTHON) -m py_compile main.py

install:
	uv venv
	@echo "Installing dependencies with uv..."
	uv pip install flake8 mypy

run:
	$(PYTHON) main.py "$(MAP)"

debug:
	$(PYTHON) -m pdb main.py "$(MAP)"

test:
	$(PYTHON) -m unittest discover -s tests -v

clean:
	@echo "Cleaning cache files..."
	rm -rf __pycache__ .mypy_cache .pytest_cache
	find . -type d -name "__pycache__" -exec rm -rf {} +
	find . -type f -name "*.pyc" -delete

lint:
	flake8 . --exclude=.venv
	mypy --warn-return-any --warn-unused-ignores --ignore-missing-imports --disallow-untyped-defs --check-untyped-defs .

lint-strict:
	flake8 . --exclude=.venv
	mypy --strict .
