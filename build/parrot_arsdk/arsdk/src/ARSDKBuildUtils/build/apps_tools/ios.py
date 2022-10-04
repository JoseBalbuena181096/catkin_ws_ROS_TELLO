# base to build an iOS application
import os
import dragon
import shutil
import re
import string
import tempfile
import tarfile
import collections

def _get_version_code_from_name(version_name):
    if version_name == "0.0.0" or version_name.startswith("0.0.0-"):
        return "0"
    if not re.match(r"[0-9]{1,2}\.[0-9]{1,2}\.[0-9]{1,2}(-(alpha|beta|rc)+[0-9]{0,2})?$",
                    version_name, flags=re.IGNORECASE):
        raise ValueError("Bad version name : " + version_name)

    try:
        (version, variant) = version_name.split("-")
    except ValueError:
        version = version_name
        variant = "release"
    (major, minor, rev) = (int(x) for x in version.split("."))

    try:
        variant_num = int(variant.strip(string.ascii_letters))
    except ValueError:
        variant_num = 0
    variant_name = variant.strip(string.digits)

    variant_codes = { "alpha": 0,
                      "beta": 1,
                      "rc": 2,
                      "release": 3,
                      };
    try:
        variant_code = variant_codes[variant_name]
    except KeyError:
        variant_code = 0

    return "{:02d}{:02d}{:02d}.{:01d}.{:02d}".format(major, minor, rev,
                                                     variant_code, variant_num)

def _set_product():
    with open(os.path.join(dragon.OUT_ROOT_DIR, "product.xcconfig"), "w") as f:
        f.write("ALCHEMY_PRODUCT = %s\n" % dragon.PRODUCT)

def _xctool(calldir, workspace, configuration, scheme,
            action, reporter, extra_args):
    cmd = "xctool"
    if (dragon.VARIANT == "ios_sim"):
        cmd += " --sdk iphonesimulator --arch x86_64"
    else:
        cmd += " --sdk iphoneos "
    cmd += " --workspace %s" % workspace
    cmd += " --configuration %s" % configuration
    cmd += " --scheme %s" % scheme
    cmd += " --reporter pretty"
    if (reporter):
        cmd += " --reporter %s" % reporter
    cmd += " %s " % action
    cmd += " ".join(extra_args)
    dragon.exec_cmd(cmd, cwd=calldir)

def add_xctool_task(calldir="", workspace="", configuration="",
                    scheme="", action="", reporter=None, extra_args=[],
                    name="", desc="", subtasks=[]):
    dragon.add_meta_task(
        name=name,
        desc=desc,
        subtasks=subtasks,
        posthook=lambda task, args: _xctool(calldir, workspace,
                                            configuration, scheme,
                                            action, reporter,
                                            extra_args)
    )

def _xcodebuild(calldir, workspace, configuration, scheme, action, bundle_id, extra_args):
    version = dragon.PARROT_BUILD_PROP_VERSION
    vname, _, _ = version.partition('-')
    vcode = _get_version_code_from_name(version)
    cmd = "xcodebuild"
    if (dragon.VARIANT == "ios_sim"):
        cmd += " -sdk iphonesimulator -arch x86_64"
    else:
        cmd += " -sdk iphoneos"
    if workspace.endswith("xcworkspace"):
        cmd += " -workspace %s" % workspace
    else:
        cmd += " -project %s" % workspace
    cmd += " -configuration %s" % configuration
    cmd += " -scheme %s" % scheme
    if os.environ.get("MOVE_APPSDATA_IN_OUTDIR"):
        cmd += " -derivedDataPath %s" % os.path.join(dragon.OUT_DIR, "xcodeDerivedData")
    cmd += " %s" % action
    cmd += " ALCHEMY_OUT=%s" % dragon.OUT_DIR
    cmd += " ALCHEMY_OUT_ROOT=%s" % dragon.OUT_ROOT_DIR
    cmd += " ALCHEMY_PRODUCT=%s" % dragon.PRODUCT
    if bundle_id:
        cmd += " PRODUCT_BUNDLE_IDENTIFIER=%s" % bundle_id
    cmd += " APP_VERSION_SHORT=%s" % vname
    cmd += " APP_VERSION=%s" % version
    cmd += " APP_BUILD=%s " % vcode
    cmd += " ".join(extra_args)
    if not dragon.OPTIONS.verbose and shutil.which("xcpretty"):
        cmd += " | xcpretty && exit ${PIPESTATUS[0]}"
    dragon.exec_cmd(cmd, cwd=calldir)

def add_xcodebuild_task(calldir="", workspace="", configuration="",
                        scheme="", action="", bundle_id=None, extra_args=[],
                        name="", desc="", subtasks=[], prehook=None):
    dragon.add_meta_task(
        name=name,
        desc=desc,
        subtasks=subtasks,
        prehook=prehook,
        posthook=lambda task, args: _xcodebuild(calldir, workspace,
                                                configuration, scheme,
                                                action, bundle_id, extra_args)
    )

def _jazzy(calldir, workspace, scheme, extra_args):
    cmd = "jazzy"
    cmd += " -x -workspace,%s" % workspace
    cmd += " -x -scheme,%s" % scheme
    outdir = os.path.join(dragon.OUT_DIR, "docs")
    cmd += " -o %s " % outdir
    cmd += " ".join(extra_args)
    dragon.exec_cmd(cmd, cwd=calldir)

def add_jazzy_task(calldir="", workspace="", scheme="", extra_args=[],
                       name="", desc="", subtasks=[]):
    dragon.add_meta_task(
        name=name,
        desc=desc,
        subtasks=subtasks,
        posthook=lambda task, args: _jazzy(calldir, workspace, scheme,
                                           extra_args)
    )

def add_task_build_common():
    dragon.add_alchemy_task(
        name="build-common",
        desc="Build ios common code",
        product=dragon.PRODUCT,
        variant=dragon.VARIANT,
        defargs=["all", "sdk"],
        weak=True,
        posthook=lambda task, args: _set_product()
    )

    dragon.add_alchemy_task(
        name="clean-common",
        desc="Clean ios common code",
        product=dragon.PRODUCT,
        variant=dragon.VARIANT,
        defargs=["clobber"],
        weak=True,
    )

_inhouse_plist_template="""
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>method</key>
    <string>enterprise</string>
    <key>teamID</key>
    <string>{}</string>
    <key>provisioningProfiles</key>
    <dict>
        <key>{}</key>
        <string>{}</string>
    </dict>
    <key>compileBitcode</key>
    <true/>
</dict>
</plist>
"""

_release_plist_template="""
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>method</key>
    <string>app-store</string>
    <key>teamID</key>
    <string>{}</string>
    <key>provisioningProfiles</key>
    <dict>
        <key>{}</key>
        <string>{}</string>
    </dict>
</dict>
</plist>
"""

def _create_export_plist(signing_infos, bundle_id, inhouse=True):
    plist_name = os.path.join(dragon.OUT_DIR, "export.plist")
    with open(plist_name, "w") as f:
        if inhouse:
            template = _inhouse_plist_template
        else:
            template = _release_plist_template
        f.write(template.format(signing_infos.team_id, bundle_id, signing_infos.profile))
    return plist_name

def _replace_app_prefix_in_entitlements(entitlements_path, signing_infos):
    fh, tmp_path = tempfile.mkstemp()
    with open(tmp_path,"w") as f, open(entitlements_path) as ent:
        for l in ent:
            f.write(re.sub(r'(.*<string)>[A-Z0-9]{10}(.com.parrot)',
                           r'\1>{}\2'.format(signing_infos.app_prefix),
                           l))
    os.close(fh)
    os.remove(entitlements_path)
    shutil.move(tmp_path, entitlements_path)

def _export_archive(dirpath, archive_path, app, entitlements_path,
                    inhouse=True):
    signing_infos = app.inhouse_infos if inhouse else app.appstore_infos
    if signing_infos is None:
        return None

    export_plist = _create_export_plist(signing_infos, app.bundle_id, inhouse=inhouse)
    _replace_app_prefix_in_entitlements(entitlements_path, signing_infos)
    ipa_path = os.path.join(dragon.OUT_DIR, "xcodeApps",
                                "inhouse" if inhouse else "appstore")
    cmd = "xcodebuild -exportArchive -archivePath {} -exportOptionsPlist {}" \
          " -exportPath {}".format(archive_path, export_plist, ipa_path)
    dragon.exec_cmd(cwd=dirpath, cmd=cmd)
    return "{}/{}.ipa".format(ipa_path, app.scheme)

def _hook_pre_images(task, args):
    # cleanup
    dragon.exec_cmd(cwd=dragon.OUT_DIR, cmd="rm -rf images")
    dragon.exec_cmd(cwd=dragon.OUT_DIR, cmd="rm -rf xcodeApps")
    manifest_path = os.path.join(dragon.OUT_DIR, "manifest.xml")
    dragon.gen_manifest_xml(manifest_path)
    task.call_base_pre_hook(args)


SignatureInfos = collections.namedtuple('SignatureInfos', ['app_prefix', 'team_id', 'profile'])

class App:
    def __init__(self, scheme, configuration, bundle_id, args=[], inhouse_infos=None, appstore_infos=None):
        self.configuration = configuration
        self.scheme = scheme
        self.bundle_id = bundle_id
        self.args = args
        self.inhouse_infos = inhouse_infos
        self.appstore_infos = appstore_infos

    def _archivePath(self, out, skipExt=False):
        path = os.path.join(out, "xcodeArchives", "{}-{}".format(self.scheme, self.configuration))
        if skipExt:
            return path
        return "{}{}".format(path, ".xcarchive")

    def _taskName(self):
        return "build-archive-{}-{}".format(self.scheme, self.configuration)
    def _taskDesc(self):
        return "build archive {}-{} for release".format(self.scheme, self.configuration)

def _make_hook_images(calldir, apps):
    def _hook_images(task, args):

        images_dir = os.path.join(dragon.OUT_DIR, "images")
        dragon.makedirs(images_dir)

        for app in apps:
            archive_path = app._archivePath(dragon.OUT_DIR)
            # Compress .xcarchive
            archive_dir = os.path.dirname(archive_path)
            archive_name = os.path.basename(archive_path)
            tarname = os.path.join(
                images_dir,
                "{}.tar.gz".format(os.path.basename(archive_path)))
            cwd = os.getcwd()
            os.chdir(archive_dir)
            tar = tarfile.open(tarname, "w:gz")
            tar.add(archive_name)
            tar.close()
            os.chdir(cwd)

            entitlements_path = os.path.join(archive_path, "Products",
                                             "Applications",
                                             "{}.app".format(app.scheme),
                                             "archived-expanded-entitlements.xcent")
            inhouse_path = _export_archive(calldir,
                                           archive_path,
                                           app,
                                           entitlements_path,
                                           inhouse=True)
            appstore_path = _export_archive(calldir,
                                            archive_path,
                                            app,
                                            entitlements_path,
                                            inhouse=False)
            # Link .ipa
            if inhouse_path:
                dragon.exec_cmd(cwd=images_dir,
                                cmd="ln -s {} {}-inhouse.ipa".format(inhouse_path,
                                                                     app.scheme))
            if appstore_path:
                dragon.exec_cmd(cwd=images_dir,
                                cmd="ln -s {} {}-appstore.ipa".format(appstore_path,
                                                                      app.scheme))

        # build.prop
        build_prop_file = os.path.join(dragon.OUT_DIR, "staging", "etc",
                                       "build.prop")
        dragon.exec_cmd(cwd=dragon.OUT_DIR, cmd="cp %s ." % build_prop_file)

        # next hooks
        task.call_base_exec_hook(args)
    return _hook_images

def _make_rm_previous_archive(app):
    def _rm_previous_archive(task, args):
        dragon.exec_cmd('rm -rf {}'.format(app._archivePath(dragon.OUT_DIR)))
    return _rm_previous_archive


def add_release_task(calldir="", workspace="", apps=[]):
    subtasks = []
    for app in apps:
        _args = ["-archivePath {}".format(app._archivePath(dragon.OUT_DIR, skipExt=True))]
        if app.args:
            _args.extend(app.args)

        add_xcodebuild_task(
            name=app._taskName(),
            desc=app._taskDesc(),
            subtasks=["build-common"],
            prehook=_make_rm_previous_archive(app),
            calldir=calldir,
            workspace=workspace,
            configuration=app.configuration,
            scheme=app.scheme,
            bundle_id=app.bundle_id,
            action="archive",
            extra_args=_args
        )
        subtasks.append(app._taskName())

    dragon.override_meta_task(
        name="images-all",
        prehook=_hook_pre_images,
        exechook=_make_hook_images(calldir, apps)
    )

    subtasks.extend(["images-all", "gen-release-archive"])

    dragon.override_meta_task(
        name="release",
        subtasks=subtasks
    )
