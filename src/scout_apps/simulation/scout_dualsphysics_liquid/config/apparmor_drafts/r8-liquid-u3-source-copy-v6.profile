# R8 liquid U3 source-copy profile v6.
# REVIEWED_SINGLE_ATTEMPT_ONLY: v5 stopped when bwrap's --disable-userns read
# the exact host guard /proc/sys/user/max_user_namespaces. v6 permits only
# that one read; it keeps nested user namespaces disabled and adds no writable
# proc rule. sudo only drops to zrj:zrj/no supplementary groups before aa-exec.
abi <abi/4.0>,

include <tunables/global>

profile r8-liquid-u3-source-copy-v6 flags=(attach_disconnected,mediate_deleted) {
  userns create,

  /usr/bin/bwrap rix,
  /usr/bin/cp rix,
  /usr/bin/env rix,
  /usr/bin/prlimit rix,

  / r,
  /usr/ r,
  /usr/lib/** mr,
  /usr/lib64/** mr,
  /lib/** mr,
  /lib64/** mr,
  /etc/ld.so.cache r,
  /etc/ld.so.conf r,
  /etc/ld.so.conf.d/** r,

  /home/ r,
  /home/zrj/ r,
  /home/zrj/scout_liquid_lab/ r,
  /home/zrj/scout_liquid_lab/dependency/ r,
  /home/zrj/scout_liquid_lab/dependency/materialized/ r,
  /home/zrj/scout_liquid_lab/dependency/materialized/u3_source_materialization_v1_20260806T155752Z.partial/ r,
  /home/zrj/scout_liquid_lab/dependency/materialized/u3_source_materialization_v1_20260806T155752Z.partial/src/ r,
  /home/zrj/scout_liquid_lab/dependency/materialized/u3_source_materialization_v1_20260806T155752Z.partial/src/source/ r,
  /home/zrj/scout_liquid_lab/dependency/materialized/u3_source_materialization_v1_20260806T155752Z.partial/src/source/** r,
  /home/zrj/scout_liquid_lab/build/ r,
  /home/zrj/scout_liquid_lab/build/u3_source_cpu_build_20260806T173417Z.partial/ rw,
  /home/zrj/scout_liquid_lab/build/u3_source_cpu_build_20260806T173417Z.partial/output/ rw,
  /home/zrj/scout_liquid_lab/build/u3_source_cpu_build_20260806T173417Z.partial/output/** rw,

  owner /proc/** r,
  owner /proc/*/uid_map w,
  owner /proc/*/gid_map w,
  owner /proc/*/setgroups w,
  /proc/filesystems r,
  /proc/sys/kernel/overflowuid r,
  /proc/sys/kernel/overflowgid r,
  /proc/sys/user/max_user_namespaces r,
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
  mount options=(rw, rbind) /oldroot/home/zrj/scout_liquid_lab/dependency/materialized/u3_source_materialization_v1_20260806T155752Z.partial/src/source/ -> /newroot/work/input/,
  remount options=(ro, nosuid, nodev, bind, silent, relatime) /newroot/work/input/,
  mount options=(rw, rbind) /oldroot/home/zrj/scout_liquid_lab/build/u3_source_cpu_build_20260806T173417Z.partial/output/ -> /newroot/work/output/,
  remount options=(rw, nosuid, nodev, bind, silent, relatime) /newroot/work/output/,

  /tmp/newroot/ rw,
  /tmp/newroot/** rw,
  /tmp/oldroot/ rw,
  /tmp/oldroot/** rw,
  /newroot/usr/ rw,
  /newroot/lib64 wl,
  /newroot/work/ rw,
  /newroot/work/input/ rw,
  /newroot/work/output/ rw,
  /work/input/ r,
  /work/input/** r,
  /work/output/ rw,
  /work/output/** rw,

  capability sys_admin,
  capability sys_ptrace,
  capability setpcap,
  capability net_admin,
  ptrace (read, readby) peer=r8-liquid-u3-source-copy-v6,
  signal (send, receive) peer=r8-liquid-u3-source-copy-v6,
  network unix dgram,
  network inet dgram,
  network inet6 dgram,
  network netlink raw,

  # Intentionally absent: every other /proc/sys path or write permission,
  # guest tmpfs /tmp, proc/dev guest mount, other executables/host binds,
  # stream networking, GPU, workspace/ROS, source write and upstream execution.
}
