%% Perfect Match Case
DT_Output = {
    "time"   "depth" "vertSpeed" "pitchUp" "pitchPos" "roll" "rollPos" "horizSpeed" "glideAngle" "buoyancy" "VBDpos" "heading";
     0        10       1.0         5          20         0      0         15            -2           100        50       90;
     1        12       1.2         6          22         1      1         16            -1           102        51       91;
     2        14       1.3         7          23         2      2         17             0           103        52       92;
     3        16       1.5         8          25         3      3         18             1           105        53       93
};

MI_Output = DT_Output;   % perfect match

missioncompare(DT_Output, MI_Output)

%% 2% Error Case


DT_Output = {
    "time"   "depth" "vertSpeed" "pitchUp" "pitchPos" "roll" "rollPos" "horizSpeed" "glideAngle" "buoyancy" "VBDpos" "heading";
     0        10       1.0         5          20         0      0         15            -2           100        50       90;
     1        12       1.2         6          22         1      1         16            -1           102        51       91;
     2        14       1.3         7          23         2      2         17             0           103        52       92;
     3        16       1.5         8          25         3      3         18             1           105        53       93
};

MI_Output = {
    "time"   "depth" "vertSpeed" "pitchUp" "pitchPos" "roll" "rollPos" "horizSpeed" "glideAngle" "buoyancy" "VBDpos" "heading";
     0        10.20    1.020       5.10       20.40      0       0        15.30        -2.04        102.00     51.00     91.80;
     1        12.24    1.224       6.12       22.44      1.02    1.02     16.32        -1.02        104.04     52.02     92.82;
     2        14.28    1.326       7.14       23.46      2.04    2.04     17.34         0           105.06     53.04     93.84;
     3        16.32    1.530       8.16       25.50      3.06    3.06     18.36         1.02        107.10     54.06     94.86
};
missioncompare(DT_Output, MI_Output)

%% Depth 50% Error Case

DT_Output = {
    "time"   "depth" "vertSpeed" "pitchUp" "pitchPos" "roll" "rollPos" "horizSpeed" "glideAngle" "buoyancy" "VBDpos" "heading";
     0        10       1.0         5          20         0      0         15            -2           100        50       90;
     1        12       1.2         6          22         1      1         16            -1           102        51       91;
     2        14       1.3         7          23         2      2         17             0           103        52       92;
     3        16       1.5         8          25         3      3         18             1           105        53       93
};

MI_Output = {
    "time"   "depth" "vertSpeed" "pitchUp" "pitchPos" "roll" "rollPos" "horizSpeed" "glideAngle" "buoyancy" "VBDpos" "heading";
     0         5       1.0         2.5          20         0      0         15            -2           100        50       90;
     1         6       1.2         3          22         1      1         16            -1           102        51       91;
     2         7       1.3         3.5          23         2      2         17             0           103        52       92;
     3         8       1.5         4          25         3      3         18             1           105        53       93
};

missioncompare(DT_Output, MI_Output)


