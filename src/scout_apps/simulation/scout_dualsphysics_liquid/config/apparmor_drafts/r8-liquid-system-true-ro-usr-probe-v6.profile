# R8 liquid target transient AppArmor read-only /usr bind probe v6.
# EPHEMERAL_PROBE_ONLY: LOAD_FROM_WORKSPACE_AND_REMOVE_IN_THE_SAME_SESSION.
# This named profile has no attachment path, so it cannot affect an existing
# process unless that process explicitly selects this exact profile.
#
# Scope: v5 plus a request for one synthetic `lib64 -> usr/lib64` symlink, as
# required by the read-only inspected ELF interpreter of /usr/bin/true. This
# v6 intentionally has NO link/create permission, so it must fail closed
# before the link exists. No host path beyond the already-read-only /usr bind,
# no writable bind, no generic mount/remount/umount, no workspace/source/output
# path, and no upstream executable is admitted. It must never be copied to
# /etc/apparmor.d.
abi <abi/4.0>,

include <tunables/global>

profile r8-liquid-system-true-ro-usr-probe-v6 flags=(attach_disconnected,mediate_deleted) {
  userns create,

  /usr/bin/bwrap rix,
  /usr/bin/true rix,

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

  /tmp/newroot/ rw,
  /tmp/newroot/** rw,
  /tmp/oldroot/ rw,
  /tmp/oldroot/** rw,
  /newroot/usr/ rw,

  capability sys_admin,
  capability sys_ptrace,
  capability setpcap,
  capability net_admin,
  ptrace (read, readby) peer=r8-liquid-system-true-ro-usr-probe-v6,
  signal (send, receive) peer=r8-liquid-system-true-ro-usr-probe-v6,
  network unix dgram,
  network inet dgram,
  network inet6 dgram,
  network netlink raw,

  # Intentionally absent: all link rules, all other bind/rbind/remount rules,
  # generic umount, source/workspace/output rule, broad host read/write,
  # /proc mount, /dev mount, stream socket, dbus, unconfined transition, or
  # upstream executable permission.
}
