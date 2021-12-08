#!/bin/python3

# TODO args

import os
from datetime import datetime

dateTimeObj = datetime.now()
timestampStr = dateTimeObj.strftime("%Y%m%d_%H%M%S")
print('Current Timestamp : ', timestampStr)

mouse_logs_dir = "benchmark/mouse_log"
os.makedirs(mouse_logs_dir, exist_ok=True)

mouse_log_name = timestampStr + "_mouse.log"
mouse_log_file = mouse_logs_dir + "/" + mouse_log_name

# RECORD
print("recording into", mouse_log_file)
os.system("./compile_release.sh;")
os.system(
        "./bin/benchmark -viewer --mode 0 -verbose-mouse ../../SDF2SVO/tmp.oct | tee " + mouse_log_file
        )


# READ RAW OUTPUT AND CONVERT TO CSV
f = open(mouse_log_file, "r")
f.seek(0)
lines = f.readlines()
f.close()

keep = [
        'SDL_event',
        'mx',
        'my',
        'getMouseDown(0)',
        'getMouseDown(1)'
        ]
col_names = [
        'SDL_event',
        'mx',
        'my',
        'getMouseDown_0',
        'getMouseDown_1'
        ]

new_lines = []
for i,l in enumerate(lines):
    for k in keep:
        if k in l:
            new_lines.append(l.strip());
            break;

# Compact

SDL_event      = ""
mx             = ""
my             = ""
getMouseDown_0 = ""
getMouseDown_1 = ""

csv_lines = [",".join(col_names)]
for i,l in enumerate(new_lines):
    if (i>0 and i%len(keep) == 0 and i>0):
        csv_lines.append(
                ",".join([
                    SDL_event,
                    mx,
                    my,
                    getMouseDown_0,
                    getMouseDown_1
                    ])
                )

    s = l.split();
    if keep[0] == s[0]:
        SDL_event = s[1];
    elif keep[1] == s[0]:
        mx = s[1];
    elif keep[2] == s[0]:
        my = s[1];
    elif keep[3] == s[0]:
        getMouseDown_0 = s[1];
    elif keep[4] == s[0]:
        getMouseDown_1 = s[1];
    else:
        print("NOT to keep ", l);

#csv_lines[0:10]
csv_lines = [l + "\n" for l in csv_lines]

# Save CSV
csv_file = mouse_log_file.replace(".log",".csv")
f = open(csv_file, "w")
f.writelines(csv_lines)
f.close()

print("mouse record save in", csv_file)

