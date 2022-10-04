#!/bin/sh

rm -rf gen
python ../ARSDKBuildUtils/Utils/Python/ARSDK_PrebuildActions.py --lib libARStream2 --root . || exit 1
