import sqlite3

def create_database():
    conn = sqlite3.connect('database/robot_data.db')
    cursor = conn.cursor()
    
    cursor.execute('''
    CREATE TABLE IF NOT EXISTS user (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        username TEXT NOT NULL UNIQUE,
        password TEXT NOT NULL
    )
    ''')

    #inserta un usuario Socialtech con contraseña 48007
    cursor.execute('''
    INSERT OR IGNORE INTO user (username, password)
    VALUES ('socialtech', '48007')
    ''')

    conn.commit()
    conn.close()

if __name__ == '__main__':
    create_database()
