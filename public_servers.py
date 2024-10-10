from flask import Flask, request, jsonify
import sqlite3

app = Flask(__name__)

@app.route('/servers', methods=['GET'])
def get_servers():
    connection = sqlite3.connect('servers.db')
    cursor = connection.cursor()
    cursor.execute("SELECT address, port, n_players FROM servers WHERE time >= strftime('%s', 'now') - 60")
    servers = [{"address": row[0], "port": row[1], "nPlayers": row[2]} for row in cursor.fetchall()]
    connection.close()
    return jsonify(servers)

@app.route('/publish', methods=['POST'])
def publish_server():
    data = request.get_json()
    connection = sqlite3.connect('servers.db')
    connection.cursor().execute("INSERT OR REPLACE INTO servers (address, port, time, n_players) VALUES (?, ?, strftime('%s', 'now'), ?)",
              (data["address"], data["port"], data["nPlayers"]))
    connection.commit()
    connection.close()
    return "", 204

if __name__ == '__main__':
    connection = sqlite3.connect('servers.db')
    connection.cursor().execute('''CREATE TABLE IF NOT EXISTS servers
                 (address TEXT, port TEXT, time INTEGER, n_players INTEGER,
                 PRIMARY KEY (address, port))''')
    connection.commit()
    connection.close()
    app.run(host='::', port=8080, debug=True)
