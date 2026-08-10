# R8 liquid U3 CPU-build profile v4.
# REVIEWED_SINGLE_ATTEMPT_ONLY: companion profile with no /proc or /dev guest
# mount, no /bin or /lib synthetic link, and fixed SHELL=/usr/bin/dash.  sudo
# only uses setpriv to drop to zrj:zrj with zero supplementary groups before
# aa-exec.  Never install this named profile under /etc/apparmor.d.
abi <abi/4.0>,

include <tunables/global>

profile r8-liquid-u3-cpu-build-v4 flags=(attach_disconnected,mediate_deleted) {
  userns create,

  /usr/bin/bwrap rix,
  /usr/bin/env rix,
  /usr/bin/prlimit rix,
  /usr/bin/make rix,
  /usr/bin/dash rix,
  /usr/bin/x86_64-linux-gnu-g++-13 rix,
  /usr/bin/x86_64-linux-gnu-as rix,
  /usr/bin/x86_64-linux-gnu-ld.bfd rix,
  /usr/bin/as rix,
  /usr/bin/ld rix,
  /usr/bin/ld.bfd rix,
  /usr/libexec/gcc/x86_64-linux-gnu/13/cc1plus rix,
  /usr/libexec/gcc/x86_64-linux-gnu/13/collect2 rix,
  /usr/lib/gcc/x86_64-linux-gnu/13/** mr,

  / r,
  /usr/ r,
  /usr/lib/** mr,
  /usr/lib64/** mr,
  /usr/libexec/** mr,
  /lib/** mr,
  /lib64/** mr,
  /etc/ld.so.cache r,
  /etc/ld.so.conf r,
  /etc/ld.so.conf.d/** r,

  /home/ r,
  /home/zrj/ r,
  /home/zrj/scout_liquid_lab/ r,
  /home/zrj/scout_liquid_lab/build/ r,
  /home/zrj/scout_liquid_lab/build/u3_source_cpu_build_20260806T172502Z.partial/ rw,
  /home/zrj/scout_liquid_lab/build/u3_source_cpu_build_20260806T172502Z.partial/output/ rw,
  /home/zrj/scout_liquid_lab/build/u3_source_cpu_build_20260806T172502Z.partial/output/** rw,

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
  mount options=(rw, rbind) /oldroot/home/zrj/scout_liquid_lab/build/u3_source_cpu_build_20260806T172502Z.partial/output/ -> /newroot/work/output/,
  remount options=(rw, nosuid, nodev, bind, silent, relatime) /newroot/work/output/,

  /tmp/newroot/ rw,
  /tmp/newroot/** rw,
  /tmp/oldroot/ rw,
  /tmp/oldroot/** rw,
  /newroot/usr/ rw,
  /newroot/lib64 wl,
  /newroot/work/ rw,
  /newroot/work/output/ rw,
  /work/output/ rw,
  /work/output/** rw,

  capability sys_admin,
  capability sys_ptrace,
  capability setpcap,
  capability net_admin,
  ptrace (read, readby) peer=r8-liquid-u3-cpu-build-v4,
  signal (send, receive) peer=r8-liquid-u3-cpu-build-v4,
  network unix dgram,
  network inet dgram,
  network inet6 dgram,
  network netlink raw,

  # Intentionally absent: proc/dev guest mounts, output artifact execution,
  # upstream binaries, source/repository/workspace binds, GPU devices, broad
  # host paths, generic mount/remount/umount, stream network, dbus,
  # profile transition and flags=(unconfined).
}
