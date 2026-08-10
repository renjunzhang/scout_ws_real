# R8 liquid target transient one-marker output-bind probe v3.
# EPHEMERAL_PROBE_ONLY: LOAD_FROM_WORKSPACE_AND_REMOVE_IN_THE_SAME_SESSION.
# This named profile has no attachment path, so it cannot affect an existing
# process unless that process explicitly selects this exact profile.
#
# Scope: v2 plus only bwrap's audited synthetic /newroot/work/output/ mountpoint
# directory. There is still NO host output source permission, output bind rule,
# or output file rule, so the marker cannot be created. No source/workspace/
# result path, generic mount/remount/umount, network stream, GPU, or persistent
# installation is allowed.
abi <abi/4.0>,

include <tunables/global>

profile r8-liquid-output-bind-probe-v3 flags=(attach_disconnected,mediate_deleted) {
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
  ptrace (read, readby) peer=r8-liquid-output-bind-probe-v3,
  signal (send, receive) peer=r8-liquid-output-bind-probe-v3,
  network unix dgram,
  network inet dgram,
  network inet6 dgram,
  network netlink raw,

  # Intentionally absent: every host output/source bind and file rule, all
  # other bind/rbind/remount rules, generic umount, /proc or /dev mount,
  # stream socket, dbus, unconfined transition, or any executable other than
  # the fixed trusted bwrap and touch binaries.
}
