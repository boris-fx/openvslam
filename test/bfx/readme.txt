== determinismTest.py ==

usage: determinismTest.py [-h] [--clipPath CLIPPATH] [--configPath CONFIGPATH] [--numReps NUMREPS] [--logLevel {trace,debug,info,warning,error,critical,off}]
                          [--printAllFrames] [--printDiff] [--keepLogs] [--build {debug,release}]
                          clipFile

Run repeated solves on a list of clips to check whether output is deterministic

positional arguments:
  clipFile              JSON file containing video clip info

optional arguments:
  -h, --help            show this help message and exit
  --clipPath CLIPPATH   directory under which clips in the clipFile can be found
  --configPath CONFIGPATH
                        directory under which config files in the clipFile can be found
  --numReps NUMREPS     number of times to repeat each solve (default = 3)
  --logLevel {trace,debug,info,warning,error,critical,off}
                        spdlog level (default = 'info')
  --printAllFrames      print results on every frame
  --printDiff           print differences between results output
  --keepLogs            keep matching generated log files after comparing instead of deleting
  --build {debug,release}
                        build config of executable to run (determines search path)

JSON clip file format:
"active_clips" will be used in the test
"extra_clips" (or any other top-level tag) is ignored by the test program
    (can be used as somewhere to save info for other clips, e.g. ones that may not work very well).

Each clip entry must have:
"clip": name of the clip file (absolute path or file in --clipPath)
"config": name of the YAML config file (absolute path or file in --configPath)

Optional parameters (for passing to run_video_slam):
"planar": directory containing planar tracks for input prematched points (absolute or relative to --trackPath)
"grid_size": size of grid to create from planar tracks
"mesh": directory containing mesh tracks for input prematched points (absolute or relative to --trackPath)

Video files etc. can be downloaded from Google Drive:
https://drive.google.com/drive/folders/1VAmT6ZooxKPf00MguODrxnoXfV9el2oJ?usp=share_link - clips
https://drive.google.com/drive/folders/1Q_UShi5dbyR-rCo64_e8yEJRdNhvAH31?usp=share_link - YAML configs
https://drive.google.com/drive/folders/1EBC-Lfhv26r2M80sR7dlBNwUKoEH4EjV?usp=share_link - Mocha tracks
