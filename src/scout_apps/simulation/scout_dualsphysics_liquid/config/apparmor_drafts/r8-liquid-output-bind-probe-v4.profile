# R8 liquid target transient one-marker output-bind probe v4.
# EPHEMERAL_PROBE_ONLY: LOAD_FROM_WORKSPACE_AND_REMOVE_IN_THE_SAME_SESSION.
# This named profile has no attachment path, so it cannot affect an existing
# process unless that process explicitly selects this exact profile.
#
# Scope: v3 plus one audited writable bind, from the sole new and empty host
# directory below to the sole synthetic target below. The bwrap argv and this
# profile contain no other writable host path. There is deliberately no rule
# for the marker file, so /usr/bin/touch must still fail closed after the bind.
#
# source: /oldroot/home/zrj/scout_liquid_lab/build/u3_apparmor_output_bind_probe_v1_20260806T151718Z/output/
# target: /newroot/work/output/
#
# No source checkout, workspace bind, result bind, generic bind/rbind/remount/
# umount, network stream, GPU, or persistent installation is allowed.
abi <abi/4.0>,

include <tunables/global>

profile r8-liquid-output-bind-probe-v4 flags=(attach_disconnected,mediate_deleted) {
  userns create,

  /usr/bin/bwrap rix,
  /usr/bin/touch rix,

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
  mount fstype=tmpfs options=(rw, nosuid, nodev) -> /newroot/work/,
  mount options=(rw, rbind) /oldroot/home/zrj/scout_liquid_lab/build/u3_apparmor_output_bind_probe_v1_20260806T151718Z/output/ -> /newroot/work/output/,

  /tmp/newroot/ rw,
  /tmp/newroot/** rw,
  /tmp/oldroot/ rw,
  /tmp/oldroot/** rw,
  /newroot/usr/ rw,
  /newroot/lib64 wl,
  /newroot/work/ rw,
  /newroot/work/output/ rw,

  capability sys_admin,
  capability sys_ptrace,
  capability setpcap,
  capability net_admin,
  ptrace (read, readby) peer=r8-liquid-output-bind-probe-v4,
  signal (send, receive) peer=r8-liquid-output-bind-probe-v4,
  network unix dgram,
  network inet dgram,
  network inet6 dgram,
  network netlink raw,

  # Intentionally absent: every other host bind/rbind/remount rule, every
  # marker or output file permission, generic umount, source/workspace/result
  # rule, broad host read/write, /proc or /dev mount, stream socket, dbus,
  # unconfined transition, or any executable other than trusted bwrap/touch.
}
