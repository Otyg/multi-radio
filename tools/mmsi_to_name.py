import requests
from html.parser import HTMLParser
import sqlite3

class VesselParser(HTMLParser):
    def __init__(self):
        super().__init__()
        self.in_td = False
        self.current_target = None
        self.current_text = []
        self.rows = {}
 
    def handle_starttag(self, tag, attrs):
        if tag == "td":
            attrs = dict(attrs)
 
            if "data-target" in attrs:
                self.in_td = True
                self.current_target = attrs["data-target"]
                self.current_text = []
 
    def handle_data(self, data):
        if self.in_td:
            self.current_text.append(data)
 
    def handle_endtag(self, tag):
        if tag == "td" and self.in_td:
            value = "".join(self.current_text).strip()
 
            self.rows.setdefault(self.current_target, []).append(value)
 
            self.in_td = False
            self.current_target = None
            self.current_text = []

def get_name(mmsi):
    form_data={'Search.MaritimeMobileServiceIdentity':mmsi, 'viewCommand':'Search'}
    resp = requests.post(url, data=form_data)
    return resp.text

url='https://www.itu.int/mmsapp/shipstation/list'
conn = sqlite3.connect("logs/track.db") # or use :memory: to put it in RAM
cursor = conn.cursor()
parser = VesselParser()
cursor.execute("""
    SELECT COUNT(DISTINCT t.entity_id)
    FROM tracks t
    LEFT JOIN entities e
        ON e.id = t.entity_id
    WHERE e.id IS NULL
       OR e.name = e.callsign
""")

antal = cursor.fetchone()[0]
print(antal)
cursor.execute('select distinct entity_id from tracks;')
ids = cursor.fetchall()
for id in ids:
    cursor.execute('select id from entities where kind = "SEA" and id ="'+id[0]+'" and (name = callsign or name = "");')
    rows = cursor.fetchall()
    if len(rows) == 0 and len(id[0]) == 9:
        parser.feed(get_name(id[0]))
    else: 
        for row in rows:
            parser.feed(get_name(row[0]))
for vessel_id, fields in parser.rows.items():
    cursor.execute("""UPDATE entities SET name = ?, callsign = ? WHERE id = ?""", (fields[0], fields[1], fields[2]))
    conn.commit()

cursor.execute("""
    SELECT COUNT(DISTINCT t.entity_id)
    FROM tracks t
    LEFT JOIN entities e
        ON e.id = t.entity_id
    WHERE e.id IS NULL
       OR e.name = e.callsign
""")

antal = cursor.fetchone()[0]
print(antal)
conn.close()

