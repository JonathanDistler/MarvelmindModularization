sqlite3 -header -csv /home/labdesktop1/ros2_ws/rosbag2_2025_10_09-16_59_19/rosbag2_2025_10_09-16_59_19_0.db3 \
"SELECT t.name AS topic_name,
        t.type AS message_type,
        COUNT(m.id) AS message_count,
        MIN(m.timestamp) AS start_time,
        MAX(m.timestamp) AS end_time
 FROM messages m
 JOIN topics t ON m.topic_id = t.id
 GROUP BY t.name, t.type
 ORDER BY t.name;" \
> /home/labdesktop1/ros2_ws/rosbag2_2025_10_09-16_59_19/rosbag_metadata.csv
