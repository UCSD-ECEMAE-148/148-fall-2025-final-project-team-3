from flask import Flask, render_template_string
import sqlite3
import os

DB_PATH = os.path.expanduser("~/robocar_configs/ros2_ws/hazard_logs.db")

app = Flask(__name__)

HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>Hazard Log Viewer</title>
    <meta http-equiv="refresh" content="5"> <!-- auto refresh -->
    <style>
        body { font-family: Arial; margin: 40px; }
        table { border-collapse: collapse; width: 100%; }
        th, td { padding: 10px; border: 1px solid #ddd; text-align: left; }
        th { background-color: #333; color: white; }
        tr:nth-child(even) { background-color: #f2f2f2; }
    </style>
</head>
<body>
    <h1>Hazard Detection Log</h1>
    <table>
        <tr>
            <th>Timestamp (UTC)</th>
            <th>Hazard Label</th>
            <th>Latitude</th>
            <th>Longitude</th>
        </tr>
        {% for row in rows %}
        <tr>
            <td>{{ row[0] }}</td>
            <td>{{ row[1] }}</td>
            <td>{{ row[2] }}</td>
            <td>{{ row[3] }}</td>
        </tr>
        {% endfor %}
    </table>
</body>
</html>
"""

@app.route("/")
def hazard_table():
    conn = sqlite3.connect(DB_PATH)
    cur = conn.cursor()
    cur.execute("SELECT timestamp, label, latitude, longitude FROM hazard_events ORDER BY timestamp DESC;")
    rows = cur.fetchall()
    conn.close()
    return render_template_string(HTML, rows=rows)

if __name__ == "__main__":
    print("Serving on http://localhost:5000")
    app.run(host="0.0.0.0", port=5000, debug=False)
