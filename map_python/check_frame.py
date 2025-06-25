import carla, time

client = carla.Client('localhost', 2000)
world  = client.get_world()

N = 100
sim_total = 0.0
wall_start = time.time()
for _ in range(N):
    world.tick()
    sim_total += world.get_snapshot().timestamp.delta_seconds
wall_total = time.time() - wall_start

print(f"Average RTF over {N} ticks: {sim_total / wall_total:.2f}")
