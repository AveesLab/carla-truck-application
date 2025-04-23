import carla

def destroy_daf_trucks():
    client = carla.Client("localhost", 2000)
    client.set_timeout(5.0)

    world = client.get_world()
    actors = world.get_actors()

    daf_list = list(actors.filter('vehicle.daf.dafxf*'))
    trailer_list = list(actors.filter('vehicle.trailer*'))
    trucks_to_destroy = daf_list + trailer_list

    for truck in trucks_to_destroy:
        print(f"[INFO] Destroying actor id={truck.id}, type={truck.type_id}")
        truck.destroy()

    print(f"[DONE] Destroyed {len(trucks_to_destroy)} vehicles (DAF + trailers).")

if __name__ == '__main__':
    destroy_daf_trucks()