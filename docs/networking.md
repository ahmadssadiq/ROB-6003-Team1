# Connecting Mourya's laptop and the robot (ROS 2 over Tailscale)

The overhead camera runs on Mourya's laptop (Ubuntu, ROS 2 Jazzy); everything else runs on the robot's
mini PC. Both must see each other's ROS topics. The course setup limits ROS to one computer
(`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`), and Tailscale does not carry the multicast that ROS normally
uses to find other computers, so the two find each other through a discovery server on the mini PC.

## Once, on both computers

1. Install Tailscale and log in with the team's Tailscale account (see the README).
2. Note the mini PC's Tailscale address: on the mini PC, `tailscale ip -4` (it starts with `100.`).

## Every session

On the mini PC, in its own terminal, start the discovery server (leave it running):

```bash
fastdds discovery --server-id 0 --port 11811
```

On both computers, in every ROS terminal, instead of the course's `LOCALHOST` line:

```bash
export ROS_DOMAIN_ID=20
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_DISCOVERY_SERVER=<mini PC tailscale address>:11811
```

For `ros2 topic list` and `ros2 topic echo` to see the other computer's topics, also set
`export ROS_SUPER_CLIENT=TRUE` in that terminal and restart the daemon once with `ros2 daemon stop`.

## Check it

On the laptop: `ros2 run demo_nodes_cpp talker`. On the mini PC: `ros2 run demo_nodes_cpp listener`.
The listener should print the talker's messages.

These settings still have to be tried on both machines; update this page with what actually works.
