## Usage

```
cd PATH_TO_analyse
python plot_data.py [-h] [-s] [-p] [-n] [-l LINEWIDTH] [-r RANGE] [-t TIME]
                    log variable [variable ...]
python plot_multiple_file.py [-h] [-s] [-p] [-l LINEWIDTH] [-r RANGE]
                             [-t TIME] [--label LABEL [LABEL ...]]
                             [--title TITLE] [--xlabel XLABEL]
                             [--ylabel YLABEL]
                             log [log ...] variable

# for example
python plot_data.py ~/.ros/log/latest/att_1-1-stdout.log cmd
```