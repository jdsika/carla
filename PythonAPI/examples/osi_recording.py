#!/usr/bin/env python3
"""
OSI GroundTruth MCAP Recording Example
=======================================

Records an MCAP trace file containing OSI GroundTruth messages for every
simulation tick.  Requires CARLA built with ``-DENABLE_OSI=ON``.

Usage
-----
  python osi_recording.py [--host HOST] [--port PORT] [--frames N] [--output PATH]

The script connects to a running CARLA server, drives the simulation in
synchronous mode, and writes one GroundTruth message per tick into the
specified MCAP file.

To inspect the resulting file you can use the ``mcap`` CLI or Foxglove Studio.
"""

import argparse
import sys
import time

import carla


def main():
    parser = argparse.ArgumentParser(
        description="Record OSI GroundTruth to an MCAP trace file."
    )
    parser.add_argument("--host", default="127.0.0.1", help="CARLA host")
    parser.add_argument("--port", default=2000, type=int, help="CARLA port")
    parser.add_argument("--frames", default=200, type=int, help="Number of frames to record")
    parser.add_argument("--output", default="carla_osi_groundtruth.mcap", help="Output MCAP path")
    args = parser.parse_args()

    if not hasattr(carla, "OsiRecorder"):
        print(
            "ERROR: carla.OsiRecorder not available.\n"
            "Rebuild CARLA with -DENABLE_OSI=ON.",
            file=sys.stderr,
        )
        return 1

    # --- Connect to CARLA ---
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    settings = world.get_settings()

    # Enable synchronous mode so we control every tick.
    original_settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05  # 20 Hz
    world.apply_settings(settings)

    try:
        # Identify the ego vehicle (first vehicle found, or spawn one).
        vehicles = world.get_actors().filter("vehicle.*")
        if len(vehicles) == 0:
            bp = world.get_blueprint_library().filter("vehicle.tesla.model3")[0]
            spawn_points = world.get_map().get_spawn_points()
            ego = world.spawn_actor(bp, spawn_points[0])
            ego.set_autopilot(True)
            print(f"Spawned ego vehicle: {ego.type_id} (id={ego.id})")
        else:
            ego = vehicles[0]
            print(f"Using existing vehicle as ego: {ego.type_id} (id={ego.id})")

        map_name = world.get_map().name

        # --- Set up OSI recording ---
        recorder = carla.OsiRecorder()
        recorder.open(args.output, f"CARLA OSI recording - {map_name}")

        builder = carla.OsiGroundTruthBuilder()

        print(f"Recording {args.frames} frames to {args.output} ...")
        t0 = time.time()

        for frame_idx in range(args.frames):
            snapshot = world.tick()

            builder.reset()
            builder.set_timestamp(snapshot.timestamp.elapsed_seconds)
            builder.set_host_vehicle_id(ego.id)
            builder.set_map_reference(map_name)

            # Add all vehicles and walkers.
            for actor in world.get_actors():
                tid = actor.type_id
                if "vehicle." in tid or "walker." in tid:
                    builder.add_moving_object(
                        actor.id,
                        tid,
                        actor.get_transform(),
                        actor.get_velocity(),
                        actor.get_acceleration(),
                        actor.get_angular_velocity(),
                        actor.bounding_box.extent,
                        actor.bounding_box.location,
                    )
                elif "traffic.traffic_light" in tid:
                    tl = actor  # carla.TrafficLight
                    builder.add_traffic_light(
                        tl.id,
                        tl.get_transform(),
                        int(tl.state),
                    )

            weather = world.get_weather()
            builder.set_environment(
                weather.precipitation,
                weather.fog_density,
                weather.sun_altitude_angle,
                weather.sun_azimuth_angle,
            )
            builder.write_to(recorder)

            if (frame_idx + 1) % 50 == 0:
                print(f"  {frame_idx + 1}/{args.frames} frames recorded")

        recorder.close()
        elapsed = time.time() - t0
        print(
            f"Done. Wrote {recorder.frame_count} frames in {elapsed:.1f}s "
            f"({recorder.frame_count / elapsed:.0f} fps) to {args.output}"
        )

    finally:
        world.apply_settings(original_settings)

    return 0


if __name__ == "__main__":
    sys.exit(main())
