import pandas as pd
import json

def process_csv_to_json(csv_file, json_file):
    # CSV 파일 불러오기
    df = pd.read_csv(csv_file)
    
    # JSON 형태로 가공
    processed_data = []
    for index, row in df.iterrows():
        lat = row['lat']
        lon = row['lon']
        speed = row['speed']
        gps_speed = 0
        
        data = {
            "lat": lat,
            "lon": lon,
            "speed": speed,
            "gps_speed": gps_speed
        }
        processed_data.append(data)
    
    # JSON 파일로 저장
    with open(json_file, 'w') as f:
        json.dump(processed_data, f)

# CSV 파일 경로와 JSON 파일 경로 설정
csv_file = 'D:\\Desktop\\newData.csv'
json_file = 'D:\\Desktop\\newOutput.json'

# CSV 파일을 JSON 파일로 가공하여 저장
process_csv_to_json(csv_file, json_file)
