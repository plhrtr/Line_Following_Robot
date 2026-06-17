#ifndef LINE_FOLLOWING_H
#define LINE_FOLLOWING_H

/**
 * Run the line follower handler.
 * Controls the robot to stay on the line.
 */
void line_following_run();

/**
 * Initialize the line following handler.
 * Unsubscribes from previous touch sensor subscriptions.
 * Subscribes to the touch sensor to detect obstacles, so it needs to be
 * executed every time before switching to line following.
 */
void line_following_init();

#endif // !LINE_FOLLOWING_H
