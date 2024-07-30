import csv
from evtx import PyEvtxParser


def parse_evtx_to_csv(evtx_filename, csv_filename):
    parser = PyEvtxParser(evtx_filename)
    
    with open(csv_filename, 'w', newline='', encoding='utf-8') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(['message', 'time', 'src', 'level', 'file'])

        for record in parser.records():
            data = []
            xml_data = record["data"]
            xml_data = text_between_delimeters(xml_data, "<Data>msg=", "</Data>")
            data.append(text_between_delimeters(xml_data, "", ",time"))
            data.append(text_between_delimeters(xml_data, "time=", ",src"))
            data.append(text_between_delimeters(xml_data, "src=", ",tid"))
            data.append(text_between_delimeters(xml_data, "lvl=", ",file"))
            data.append(xml_data[xml_data.find("file=")+5 :])

            csvwriter.writerow(data)

            #print(f"{progress}/{total_lines}", end="\r")

def text_between_delimeters(text, delimiter1, delimiter2):
    start = text.find(delimiter1)
    if start == -1:
        return None  
    start += len(delimiter1)  
    
    end = text.find(delimiter2, start)
    if end == -1:
        return None  
    
    return text[start:end]


if __name__ == "__main__":
    parse_evtx_to_csv('Globus.evtx', 'Globus.csv')
