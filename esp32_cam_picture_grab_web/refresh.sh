# 1. 彻底销毁当前容器 + 清理残留
sudo docker compose down -v

# 2. 清理系统死锁的 scope 单元
sudo systemctl reset-failed

# 3. 强制杀掉所有残留 docker 进程
sudo pkill -f docker

# 4. 重启 Docker 服务（最关键）
sudo systemctl restart docker

# 5. 重新干净启动容器
sudo docker compose up -d

# 6. 短暂开启5000端口
sudo ufw allow 5000/tcp
