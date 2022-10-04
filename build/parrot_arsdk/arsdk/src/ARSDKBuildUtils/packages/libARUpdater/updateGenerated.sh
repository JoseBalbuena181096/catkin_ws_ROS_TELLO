#!/bin/sh

rm -rf gen
python ../ARSDKBuildUtils/Utils/Python/ARSDK_PrebuildActions.py --lib libARUpdater --root . || exit 1
