.PHONY: all test install environment debug

install:
	pip install -r requirements.txt

environment:
	(\
		echo "> Creating venv"; \

		# if packages are not installed, install them
		if ! dpkg -s python3-venv &> /dev/null || ! dpkg -s python3-tk &> /dev/null; then \
			echo "> Installing required packages"; \
			sudo apt-get update; \
			sudo apt-get install python3-venv python3-tk -y; \
		fi; \

		python3 -m venv .venv; \

		source .venv/bin/activate; \

		echo "> Installing requirements"; \
		pip install -r requirements.txt; \
	)

clean:
	echo "> Removing virtual environment [y/N]"; \
	read answer; \
	if [ "$$answer" = "y" ] || [ "$$answer" = "Y" ]; then \
		rm -rf .venv; \
		echo "> Virtual environment removed"; \
	else \
		echo "> Aborted"; \
	fi;

runCameraReel:
	python3 src/mainCameraReel.py

runCameraSimu:
	python3 src/mainCameraSimu.py

run:
	python3 src/main.py