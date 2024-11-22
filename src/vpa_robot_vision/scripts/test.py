import threading

# 初始化锁
lock = threading.Lock()

# 临界区的函数
def critical_section(thread_id):
    with lock:  # 自动加锁和解锁
        print(f"Thread {thread_id} entering critical section")
        # 模拟关键任务
        import time
        time.sleep(1)
        print(f"Thread {thread_id} leaving critical section")

# 创建多个线程
threads = [threading.Thread(target=critical_section, args=(i,)) for i in range(5)]

# 启动线程
for t in threads:
    t.start()

# 等待线程完成
for t in threads:
    t.join()
