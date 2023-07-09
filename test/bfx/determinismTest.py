#!/usr/bin/env python3
from sys import platform, argv
from subprocess import call
from filecmp import cmp
from difflib import unified_diff
import os
import argparse
import json
import re

# Line-by-line diff of two files with optional printing
def fileDiff(file1, file2, printDiff):
    diff = []
    with open(file1, 'r') as f1:
        with open(file2, 'r') as f2:
            diff = unified_diff(
                f1.readlines(),
                f2.readlines(),
                fromfile=file1,
                tofile=file2,
            )

    diffLines = []
    for line in diff:
        for prefix in ('-', '+'):
            if line.startswith(prefix):
                diffLines.append(line)
                break

    # Ignore unimportant differences (e.g. thread start/stop, async operations)
    ignore = ( os.path.basename(file1), os.path.basename(file2), \
        'start .+ module$', \
        'terminate .+ module$', \
        'resume .+ module$', \
        'reset .+ module$', \
        'tracking_module: start keyframe insertion', \
        'mapping_module::async_pause', \
        'global_optimization_module::run', \
        'Stage "Analysing video" progress', \
        'relocalization succeeded', \
        'map_selector::should_reset_map_for_tracking_failure', \
        'start loop bundle adjustment', \
        'optimizer.optimize iterations', \
        '###'
        )
    
    filesMatch = True
    for line in diffLines:
        relevant = True
        for ig in ignore:
            if re.search(ig, line):
                relevant = False
                break
        if relevant:
            filesMatch = False
            if printDiff:
                print(line, end='')
    
    return filesMatch

# Run a solve on a given video input a specified number of times and compare the output
def runTest(clipFile, configFile, extraArgs, numReps, build, allFrames, logLevel, printDiff, keepLogs):
    cmdDir = "../../../tmp/" + build + "/shared"
    vocabFile = "../../../../resources/orb_vocab.fbow"

    # Build the base run_video_slam command line
    baseCmd = cmdDir + "/run_video_slam"
    if platform == "win32":
        baseCmd += ".exe"
    baseCmd += " -v " + vocabFile + " -m \"" + clipFile + "\" -c \"" + configFile + "\""
    baseCmd += " --log-level " + logLevel + " --log-times 0 --print-results 1"
    if allFrames:
        baseCmd += " --printFrames 1"
    baseCmd += extraArgs

    # Make sure log file path doesn't exceed the character limit
    filename = os.path.basename(clipFile)
    filenameLen = len(filename)
    pathLen = len(os.getcwd()) + filenameLen
    excess = pathLen - 250 - len(str(numReps))
    if (excess > 0):
        if (excess >= filenameLen):
            print("Error: cannot create log file for '" + filename + "'; cwd path too long.")
            return False
        filename = filename[:filenameLen-excess]

    logFiles = [filename + "_" + str(i) + ".txt" for i in range(numReps)]
    prevFile = ''
    for logFile in logFiles:
        if os.path.isfile(logFile):
            print("Error - file exists: " + logFile)
            return False
        
        cmd = baseCmd + " --log-file \"" + logFile + "\""
        print("Running command: " + cmd)
        call(cmd)

        if prevFile:
            if not cmp(prevFile, logFile) and not fileDiff(prevFile, logFile, printDiff):
                print("Files differ: " + prevFile + ", " + logFile)
                return False
            print("Files match: " + prevFile + ", " + logFile)
        prevFile = logFile
    
    if not keepLogs:
        for logFile in logFiles:
            os.remove(logFile)
    
    return True
    
#---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Run repeated solves on a list of clips to check whether output is deterministic')
    parser.add_argument('clipFile',
        help='JSON file containing video clip info')
    parser.add_argument('--clipPath',
        help="directory under which clips in the clipFile can be found")
    parser.add_argument('--configPath',
        help="directory under which config files in the clipFile can be found")
    parser.add_argument('--trackPath',
        help="directory under which tracking files in the clipFile can be found")
    parser.add_argument('--numReps', type=int, default=3,
        help="number of times to repeat each solve (default = 3)")
    parser.add_argument('--logLevel', choices=['trace', 'debug', 'info', 'warning', 'error', 'critical', 'off'],
        default='info',
        help="spdlog level (default = 'info')")
    parser.add_argument('--printAllFrames', action='store_true',
        help="print results on every frame")
    parser.add_argument('--printDiff', action='store_true',
        help="print differences between results output")
    parser.add_argument('--keepLogs', action='store_true',
        help="keep matching generated log files after comparing instead of deleting")
    parser.add_argument('--build', choices=['debug', 'release'], default='debug',
        help="build config of executable to run (determines search path)")
    parser.add_argument('--testAutoFeatures', action='store_true',
        help="run tests using (only) auto-generated ORB features, as well as with tracked points")
    parser.add_argument('--testPlanarTracks', action='store_true',
        help="run tests using tracked planar points")
    parser.add_argument('--testMeshTracks', action='store_true',
        help="run tests using tracked mesh points")
    args = parser.parse_args()

    # Read the clip info from file
    clipData = []
    clipPath = args.clipPath.strip() if args.clipPath else ""
    configPath = args.configPath.strip() if args.configPath else ""
    trackPath = args.trackPath.strip() if args.trackPath else ""
    with open(args.clipFile, 'r') as file:
        data = json.load(file)
        for clipInfo in data["active_clips"]:
            if "clip" not in clipInfo or "config" not in clipInfo:
                print("Missing data for clip/config: ", end='')
                print(json.dumps(clipInfo))
                continue

            clip = os.path.join(clipPath, clipInfo["clip"].strip())
            config = os.path.join(configPath, clipInfo["config"].strip())
            if not os.path.isfile(clip):
                print("Invalid clip: " + clip)
                continue
            if not os.path.isfile(config):
                print("Invalid config: " + config)
                continue

            extraArgs = []
            if args.testAutoFeatures:
                extraArgs.append("")
            if args.testPlanarTracks and "planar" in clipInfo:
                trackDir = os.path.join(trackPath, clipInfo["planar"].strip())
                if os.path.isdir(trackDir):
                    arg = " --planar \"" + trackDir + "\""
                    if "grid_size" in clipInfo:
                        arg += " --grid-size " + clipInfo["grid_size"].strip()
                    extraArgs.append(arg)
                else:
                    print("Invalid planar tracks directory: " + trackDir)
            if args.testMeshTracks and "mesh" in clipInfo:
                trackDir = os.path.join(trackPath, clipInfo["mesh"].strip())
                if os.path.isdir(trackDir):
                    extraArgs.append(" --mesh \"" + trackDir + "\"")
                else:
                    print("Invalid mesh tracks directory: " + trackDir)

            for extra in extraArgs:
                clipData.append((clip, config, extra))
        file.close()

    for clip in clipData:
        if not runTest(clip[0], clip[1], clip[2], args.numReps, args.build,
            args.printAllFrames, args.logLevel, args.printDiff, args.keepLogs):
            exit(0)
    
    print("Determinism tests complete - all results matched.")
