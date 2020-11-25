#redis-server stop
sysctl vm.overcommit_memory=1 && redis-server redis.conf
