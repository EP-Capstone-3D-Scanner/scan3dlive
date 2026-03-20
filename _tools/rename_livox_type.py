import sqlite3
import os

def fix_sqlite_bag(bag_path):
    # Find the .db3 file inside the bag folder
    db_files = [f for f in os.listdir(bag_path) if f.endswith('.db3')]
    if not db_files:
        print("No .db3 file found!")
        return
    
    db_path = os.path.join(bag_path, db_files[0])
    print(f"Opening database: {db_path}")

    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # 1. Update the 'topics' table where ROS2 stores the type names
    old_type = 'livox_ros_driver/msg/CustomMsg'
    new_type = 'livox_ros_driver2/msg/CustomMsg'

    cursor.execute("UPDATE topics SET type = ? WHERE type = ?", (new_type, old_type))
    
    if cursor.rowcount > 0:
        print(f"Successfully updated {cursor.rowcount} topic(s) in the database.")
        conn.commit()
    else:
        print("No matching topics found in the database. Check if the type name is exact.")

    conn.close()

if __name__ == '__main__':
    # REPLACE with your folder path
    BAG_FOLDER = '/scan3dlive/datasets/Retail_Street_Ros2' 
    fix_sqlite_bag(BAG_FOLDER)