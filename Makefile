.PHONY: all test install environment debug

install:
	pip install -r requirements.txt

environment:
	(\
		echo "> Creating venv"; \
		python3 -m venv .venv; \
		source .venv/bin/activate; \
		echo "> Installing requirements"; \
		pip install -r requirements.dev.txt; \
		sudo apt-get install python3-tk -y; \
	)

clean: ## Remove virtual env
	echo "> Removing virtual environment"
	rm -r .venv

runCamera: ## Run default mode
	python3 src/mainCamera.py

runSimu:
	python3 PythonApplication2.py

run:
	python3 src/main.py;

debug: ## Run local mode
	python3 src/main.py --debug --env local