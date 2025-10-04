.PHONY: all test install environment debug

install:
	pip install -r requirements.txt

environment:
	(\
		echo "> Creating venv"; \
		sudo apt-get install python3-venv -y; \
		python3 -m venv .venv; \
		source .venv/bin/activate; \
		echo "> Installing requirements"; \
		pip install -r requirements.dev.txt; \
		sudo apt-get install python3-tk -y; \
	)

clean:
	echo "> Removing virtual environment"
	rm -r .venv

runCameraReel:
	python3 src/mainCameraReel.py

runCameraSimu:
	python3 src/mainCameraSimu.py

run:
	python3 src/main.py