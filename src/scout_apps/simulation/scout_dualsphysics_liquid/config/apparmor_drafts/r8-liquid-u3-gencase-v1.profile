# R8 liquid U3 C1 GenCase bootstrap/runtime profile v1.
# REVIEWED_SINGLE_ATTEMPT_ONLY: this profile is transient and may be loaded
# only for u3_c1_gencase_20260807T032831Z.  The bootstrap label can create the
# reviewed bwrap namespace and copy two frozen files to guest tmpfs.  The
# GenCase process must transition into the second label, which deliberately has
# no mount, user-namespace, capability, tool-execution, network or host-path
# permissions beyond the guest input/output paths below.
abi <abi/4.0>,

include <tunables/global>

profile r8-liquid-u3-gencase-bootstrap-v1 flags=(attach_disconnected,mediate_deleted) {
  userns create,

  /usr/bin/bwrap rix,
  /usr/bin/cp rix,
  /usr/bin/chmod rix,
  /usr/bin/dash rix,
  /usr/bin/env rix,
  /usr/bin/prlimit rix,
  /runtime/GenCase_linux64 rPx -> r8-liquid-u3-gencase-runtime-v1,

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
  /home/zrj/scout_liquid_lab/dependency/runtime/ r,
  /home/zrj/scout_liquid_lab/dependency/runtime/u3_c1_gencase_seed_20260807T031624Z.partial/ r,
  /home/zrj/scout_liquid_lab/dependency/runtime/u3_c1_gencase_seed_20260807T031624Z.partial/input/ r,
  /home/zrj/scout_liquid_lab/dependency/runtime/u3_c1_gencase_seed_20260807T031624Z.partial/input/** r,
  /home/zrj/scout_liquid_lab/cases/ r,
  /home/zrj/scout_liquid_lab/cases/u3_c1_gencase_20260807T032831Z.partial/ rw,
  /home/zrj/scout_liquid_lab/cases/u3_c1_gencase_20260807T032831Z.partial/output/ rw,
  /home/zrj/scout_liquid_lab/cases/u3_c1_gencase_20260807T032831Z.partial/output/** rw,

  owner /proc/** r,
  owner /proc/*/uid_map w,
  owner /proc/*/gid_map w,
  owner /proc/*/setgroups w,
  /proc/filesystems r,
  /proc/sys/kernel/overflowuid r,
  /proc/sys/kernel/overflowgid r,
  /proc/sys/user/max_user_namespaces w,
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
  mount options=(rw, rbind) /oldroot/home/zrj/scout_liquid_lab/dependency/runtime/u3_c1_gencase_seed_20260807T031624Z.partial/input/ -> /newroot/work/input/,
  remount options=(ro, nosuid, nodev, bind, silent, relatime) /newroot/work/input/,
  mount options=(rw, rbind) /oldroot/home/zrj/scout_liquid_lab/cases/u3_c1_gencase_20260807T032831Z.partial/output/ -> /newroot/work/output/,
  remount options=(rw, nosuid, nodev, bind, silent, relatime) /newroot/work/output/,
  mount fstype=tmpfs options=(rw, nosuid, nodev) -> /newroot/runtime/,

  /tmp/newroot/ rw,
  /tmp/newroot/** rw,
  /tmp/oldroot/ rw,
  /tmp/oldroot/** rw,
  /newroot/usr/ rw,
  /newroot/lib wl,
  /newroot/lib64 wl,
  /newroot/work/ rw,
  /newroot/work/input/ rw,
  /newroot/work/output/ rw,
  /newroot/runtime/ rw,
  /work/input/ r,
  /work/input/** r,
  /work/output/ rw,
  /work/output/** rw,
  /runtime/ rw,
  /runtime/** rw,

  capability sys_admin,
  capability sys_ptrace,
  capability sys_resource,
  capability setpcap,
  capability net_admin,
  ptrace (read, readby) peer=r8-liquid-u3-gencase-bootstrap-v1,
  signal (send, receive) peer=r8-liquid-u3-gencase-bootstrap-v1,
  network unix dgram,
  network inet dgram,
  network inet6 dgram,
  network netlink raw,
}

profile r8-liquid-u3-gencase-runtime-v1 flags=(attach_disconnected,mediate_deleted) {
  /runtime/GenCase_linux64 rm,
  /runtime/DsphConfig.xml r,
  / r,
  /usr/ r,
  /usr/lib/** mr,
  /usr/lib64/** mr,
  /lib/** mr,
  /lib64/** mr,
  /work/input/ r,
  /work/input/** r,
  /work/output/ rw,
  /work/output/** rw,

  # No userns, mount, capability, network, /proc, /dev, system-tool exec,
  # source/workspace bind, GPU, ROS or Gazebo rule is present in this label.
}
