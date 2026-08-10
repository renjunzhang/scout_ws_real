# R8 liquid U3 C1M CPU-solver settle cold A v7, fresh one-attempt profile.
# Campaign: u3_c1m_solver_cpu_settle_ab_campaign_20260809T051659Z
# Case: u3_c1m_solver_cpu_settle_cold_a_v7_20260809T051659Z (campaign role cold_a).
#
# Never install this file below /etc/apparmor.d.  Only the root-owned one-shot
# snapshot supervisor may load these exact bytes, and it must unload both exact
# labels during the same lifecycle.  The CPU solver is started through the
# hash-pinned guest copy of ld-linux.  It inherits the bootstrap label (rix),
# while the second label is deliberately unreachable cleanup evidence.
profile r8-liquid-u3-c1m-solver-cpu-settle-cold-a-bootstrap-v7-20260809t051659z flags=(attach_disconnected,mediate_deleted) {
  userns create,

  /usr/bin/bwrap rix,
  /usr/bin/python3.12 rix,
  /work/runtime/ld-linux-x86-64.so.2 rix,

  / r,
  /usr/ r,
  /usr/lib/** mr,
  /usr/lib64/** mr,
  /lib/** mr,
  /lib64/** mr,
  deny /etc/ld.so.cache r,

  /work/ rw,
  /work/** rw,
  /work/runtime/ld-linux-x86-64.so.2 mr,
  /work/runtime/DualSPHysics5.4CPU_linux64 mr,
  /work/runtime/lib/** mr,

  # Exact directory read needed only for os.scandir('/proc').  The numeric
  # stat rule is the sole v7 authority delta, justified by the two frozen v6
  # /proc/3/stat DENIED records (fsuid=1000, ouid=0).  No status, broad glob,
  # write, or other non-owner /proc authority is granted.
  /proc/ r,
  /proc/[0-9]*/stat r,
  owner /proc/** r,
  owner /proc/*/uid_map w,
  owner /proc/*/gid_map w,
  owner /proc/*/setgroups w,
  /proc/filesystems r,
  /proc/sys/kernel/overflowuid r,
  /proc/sys/kernel/overflowgid r,
  /proc/sys/user/max_user_namespaces w,
  /proc/stat r,

  # Exact bwrap mount grammar inherited from the frozen v11 lifecycle harness.
  # There is no host-writable bind, --file, --bind-fd, device or sysfs mount.
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
  mount fstype=proc options=(rw, nosuid, nodev, noexec) proc -> /newroot/proc/,

  /tmp/newroot/ rw,
  /tmp/newroot/** rw,
  /tmp/oldroot/ rw,
  /tmp/oldroot/** rw,
  /newroot/usr/ rw,
  /newroot/lib wl,
  /newroot/lib64 wl,
  /newroot/work/ rw,
  /newroot/proc/ rw,

  deny capability dac_override,
  capability sys_admin,
  capability sys_ptrace,
  capability sys_resource,
  capability setpcap,
  capability net_admin,
  ptrace (read, readby) peer=r8-liquid-u3-c1m-solver-cpu-settle-cold-a-bootstrap-v7-20260809t051659z,
  signal (send,receive) set=(term,kill,exists) peer=r8-liquid-u3-c1m-solver-cpu-settle-cold-a-bootstrap-v7-20260809t051659z,
  signal (receive) set=(term,kill,exists) peer=unconfined,
  network unix dgram,
  network inet dgram,
  network inet6 dgram,
  network netlink raw,

  # Intentionally absent: host home/workspace/build/case paths, writable host
  # binds, /dev, /sys, GPU, ROS/Gazebo, external network, persistent profiles,
  # profile transitions, and generic mount authority.
}

profile r8-liquid-u3-c1m-solver-cpu-settle-cold-a-runtime-v7-20260809t051659z flags=(attach_disconnected,mediate_deleted) {
  # Deliberately unreachable.  The copied loader uses rix under NNP and remains
  # in the bootstrap label.  This empty label exists only to close cleanup
  # evidence over a fresh pair of names.
}
