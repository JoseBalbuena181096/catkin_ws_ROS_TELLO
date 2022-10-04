#!/bin/sh

rm -rf gen
python ../ARSDKBuildUtils/Utils/Python/ARSDK_PrebuildActions.py --lib libARMavlink --root . || exit 1

