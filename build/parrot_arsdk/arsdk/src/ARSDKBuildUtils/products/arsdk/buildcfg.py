import sys, os
import dragon

import apps_tools.android as android
import apps_tools.ios as ios

android_arsdk3_dir = os.path.join(dragon.WORKSPACE_DIR,
        "packages", "ARSDK3")

android_sample_dir = os.path.join(dragon.WORKSPACE_DIR,
        "packages", "Samples", "Android", "SDKSample")

ios_sample_dir = os.path.join(dragon.WORKSPACE_DIR,
        "packages", "Samples", "iOS", "SDKSample")

#===============================================================================
# Android
#===============================================================================

if dragon.VARIANT == "android":
    android_abis = ["armeabi", "armeabi-v7a", "arm64-v8a",
                    "mips",
                    "x86"]

    android.add_task_build_common(android_abis)

    android.add_ndk_build_task(
        name="build-jni",
        desc="Build android common code & arsdk jni",
        subtasks=["build-common"],
        calldir=os.path.join(android_arsdk3_dir, "arsdk", "jni"),
        module="arsdk",
        abis=android_abis,
        extra_args=["PACKAGES_DIR={}".format(os.path.join(dragon.WORKSPACE_DIR,
                                                          "packages"))]
    )

    android.add_ndk_build_task(
        name="clean-jni",
        desc="Clean android arsdk jni",
        calldir=os.path.join(android_arsdk3_dir, "arsdk", "jni"),
        module="arsdk",
        abis=android_abis,
        extra_args=["PACKAGES_DIR={}".format(os.path.join(dragon.WORKSPACE_DIR,
                                                          "packages")),
                    "clean"],
        ignore_failure=True
    )

    dragon.add_meta_task(
        name="build-sdk",
        desc="build android sdk",
        subtasks=["build-jni"]
    )

    dragon.add_meta_task(
        name="clean-sdk",
        desc="clean android sdk",
        subtasks=["clean-jni", "clean-common"]
    )

    android.add_gradle_task(
        name="publish",
        desc="Build & publish android sdk & jni",
        subtasks=["build-sdk"],
        calldir=android_arsdk3_dir,
        target="bintrayUpload"
    )

    if os.path.exists(android_sample_dir):
        android.add_gradle_task(
            name="build-sample",
            desc="Build the android sample in debug",
            subtasks=["build-sdk"],
            calldir=os.path.join(android_sample_dir, "buildWithLocalSDK"),
            target="assembleDebug"
        )
        android.add_gradle_task(
            name="clean-sample",
            desc="Clean the android sample",
            subtasks=["clean-sdk"],
            calldir=os.path.join(android_sample_dir, "buildWithLocalSDK"),
            target="clean"
        )

#===============================================================================
# iOS
#===============================================================================
if dragon.VARIANT == "ios" or dragon.VARIANT == "ios_sim":
    ios.add_task_build_common()

    dragon.add_meta_task(
        name="build-sdk",
        desc="build ios sdk",
        subtasks=["build-common"]
    )

    dragon.add_meta_task(
        name="clean-sdk",
        desc="clean ios sdk",
        subtasks=["clean-common"]
    )

    if os.path.exists(ios_sample_dir):
        ios.add_xcodebuild_task(
            name="build-sample",
            desc="Build the ios samples in debug",
            subtasks=["build-sdk"],
            calldir=ios_sample_dir,
            workspace="SDKSample.xcodeproj",
            configuration="DebugWithLocalSDK",
            scheme="SDKSample",
            action="build"
        )
        ios.add_xcodebuild_task(
            name="clean-sample",
            desc="Clean the ios samples in debug",
            subtasks=["clean-sdk"],
            calldir=ios_sample_dir,
            workspace="SDKSample.xcodeproj",
            configuration="DebugWithLocalSDK",
            scheme="SDKSample",
            action="clean"
        )


#===============================================================================
# Unix
#===============================================================================

def add_unix_sample(sample):
    name = "build-sample-%s" % sample
    c_name = "clean-sample-%s" % sample
    dragon.add_alchemy_task(
        name=name,
        desc="Build unix sdk sample for %s" % sample,
        product=dragon.PRODUCT,
        variant=dragon.VARIANT,
        defargs=[sample],
    )
    dragon.add_alchemy_task(
        name=c_name,
        desc="Clean unix sdk sample for %s" % sample,
        product=dragon.PRODUCT,
        variant=dragon.VARIANT,
        defargs=["%s-clean" % sample],
    )
    return (name, c_name)

if dragon.VARIANT == "native":
    dragon.add_alchemy_task(
        name="build-sdk",
        desc="Build native sdk",
        product=dragon.PRODUCT,
        variant=dragon.VARIANT,
        defargs=["all"]
    )
    dragon.add_alchemy_task(
        name="clean-sdk",
        desc="Clean native sdk",
        product=dragon.PRODUCT,
        variant=dragon.VARIANT,
        defargs=["clobber"]
    )

    all_samples = []
    clean_samples = []
    samples = ["BebopSample", "JumpingSumoSample"]
    for sample in samples:
        b, c = add_unix_sample(sample)
        all_samples.append(b)
        clean_samples.append(c)

    dragon.add_meta_task(
        name="build-sample",
        desc="Build all native samples",
        subtasks=["build-sdk"] + all_samples
    )

    dragon.add_meta_task(
        name="clean-sample",
        desc="Clean all native samples",
        subtasks=clean_samples + ["clean-sdk"]
    )

#===============================================================================
# generate sources task
#===============================================================================
def hook_gen_sources(task, args):
    packages_dir = os.path.join(dragon.WORKSPACE_DIR, "packages")
    for package in os.listdir(packages_dir):
        try:
            path = os.path.join(packages_dir, package)
            if os.path.isfile(os.path.join(path, "updateGenerated.sh")):
                dragon.exec_cmd(cmd="./updateGenerated.sh", cwd=path)
                dragon.exec_cmd(cmd="git status", cwd=path)
        except dragon.TaskError as ex:
            dragon.logging.error(str(ex))

dragon.add_meta_task(
    name="gensources",
    desc="Generate all sdk sources",
    posthook=hook_gen_sources,
)

dragon.add_meta_task(
    name="build",
    desc="Build sdk & samples",
    subtasks=["build-sample"]
)

dragon.add_meta_task(
    name="clean",
    desc="Clean sdk & samples",
    subtasks=["clean-sample"]
)
