# R8 liquid target transient one-marker output-bind probe v1.
# EPHEMERAL_PROBE_ONLY: LOAD_FROM_WORKSPACE_AND_REMOVE_IN_THE_SAME_SESSION.
# This named profile has no attachment path, so it cannot affect an existing
# process unless that process explicitly selects this exact profile.
#
# The fixed bwrap argv names exactly one new, empty host output directory:
# /home/zrj/scout_liquid_lab/build/u3_apparmor_output_bind_probe_v1_20260806T151718Z/output
# and exactly one child target: /work/output/.r8_output_bind_probe_v1.
# This v1 intentionally grants NO /work tmpfs or output bind rule, and no
# permission for that host output path. It must therefore fail closed before
# the output directory is mounted or /usr/bin/touch can write a marker.
# It has no source, workspace, result, ROS, GPU, generic mount/remount/umount,
# network stream, or persistent-install permission. It must never be copied to
# /etc/apparmor.d.
abi <abi/4.0>,

include <tunables/global>

profile r8-liquid-output-bind-probe-v1 flags=(attach_disconnected,mediate_deleted) {
  userns create,

  # The only executable chain is aa-exec -> bwrap -> touch with a literal
  # marker argument fixed in the command below; no shell or generic command
  # argument is admitted.
  /usr/bin/bwrap rix,
  /usr/bin/touch rix,

  # Required by the trusted bwrap and touch loader paths. The only host tree
  # intended for a later read-only bind is /usr; no other host prefix appears.
  / r,
  /usr/ r,
  /usr/lib/** mr,
  /usr/lib64/** mr,
  /lib/** mr,
  /lib64/** mr,
  /etc/ld.so.cache r,
  /etc/ld.so.conf r,
  /etc/ld.so.conf.d/** r,

  owner /proc/** r,
  owner /proc/*/uid_map w,
  owner /proc/*/gid_map w,
  owner /proc/*/setgroups w,
  /proc/filesystems r,
  /proc/sys/kernel/overflowuid r,
  /proc/sys/kernel/overflowgid r,
  /dev/null rw,
  /dev/zero r,
  /dev/random r,
  /dev/urandom r,

  # Previously audited internal empty-root setup and the exact read-only /usr
  # bind path. These paths are all inside bwrap's new mount namespace.
  mount options=(rw, silent, rslave) -> /,
  mount fstype=tmpfs options=(rw, nosuid, nodev) -> /tmp/,
  mount options=(rw, rbind) /tmp/newroot/ -> /tmp/newroot/,
  pivot_root oldroot=/tmp/oldroot/ /tmp/,
  mount options=(rw, silent, rprivate) -> /oldroot/,
  umount /oldroot/,
  pivot_root oldroot=/newroot/ /newroot/,
  umount /,
  mount options=(rw, rbind) /oldroot/usr/ -> /newroot/usr/,
  remount options=(ro, nosuid, nodev, bind, silent, relatime) /newroot/usr/,

  /tmp/newroot/ rw,
  /tmp/newroot/** rw,
  /tmp/oldroot/ rw,
  /tmp/oldroot/** rw,
  /newroot/usr/ rw,
  /newroot/lib64 wl,

  capability sys_admin,
  capability sys_ptrace,
  capability setpcap,
  capability net_admin,
  ptrace (read, readby) peer=r8-liquid-output-bind-probe-v1,
  signal (send, receive) peer=r8-liquid-output-bind-probe-v1,
  network unix dgram,
  network inet dgram,
  network inet6 dgram,
  network netlink raw,

  # Intentionally absent: /work tmpfs, any output/source bind, every host
  # output file rule, all other bind/rbind/remount rules, generic umount,
  # /proc or /dev mount, stream socket, dbus, unconfined transition, or any
  # executable other than the fixed trusted bwrap and touch binaries.
}
