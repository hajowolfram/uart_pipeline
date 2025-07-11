## 1. clone repo
```bash
git clone https://github.com/hajowolfram/uart_pipeline.git
```
## 2. setup
### for unix:
list ports
```bash
ls /dev/tty.* /dev/cu.*
```
rename .env_dummy to .env
modify the path and ports in .env
### for windows (NOTE, WINDOWS NOT YET SUPPORTED):
check device manager -> ports: COM & LPT

## 3. setting up virtual environment
create virtual environment of choice (eg. using conda)
```bash
conda create --name "your-env-name" python=3.8
```
activate virtual environment
```bash 
conda activate "your-env-name"
```
install dependencies
```bash
pip install -r requirements.txt
```

## 4. running the script:
run script (writing to tlv to data/ in repo root)
```bash
python main.py
```

## 5. optional custom output formatting
see write_frame_to_file in parser.h to customise formatting as desired

## additional resources
3D people tracking:
https://dev.ti.com/tirex/explore/node?node=A__AVHQ2zNotxcg9DLorsWfAA__radar_toolbox__1AslXXD__LATEST

See section 3 for TLV docs:
TODO