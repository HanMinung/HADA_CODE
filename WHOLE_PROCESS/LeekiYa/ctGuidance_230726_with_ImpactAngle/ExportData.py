import csv
import os



def col1_to_CSV(list1, filename):
    # 3개의 리스트를 zip하여 행으로 묶음
    rows = zip(list1)

    # CSV 파일 열기
    with open(filename, 'w', newline='') as file:
        # CSV writer 생성
        csv_writer = csv.writer(file)

        # 각 행을 CSV 파일에 작성
        for row in rows:
            csv_writer.writerow(row)

def col2_to_CSV(list1,list2, filename):
    # 3개의 리스트를 zip하여 행으로 묶음
    rows = zip(list1,list2)

    # CSV 파일 열기
    with open(filename, 'w', newline='') as file:
        # CSV writer 생성
        csv_writer = csv.writer(file)

        # 각 행을 CSV 파일에 작성
        for row in rows:
            csv_writer.writerow(row)

def col3_to_CSV(list1,list2,list3, filename):
    # 3개의 리스트를 zip하여 행으로 묶음
    rows = zip(list1,list2,list3)

    # CSV 파일 열기
    with open(filename, 'w', newline='') as file:
        # CSV writer 생성
        csv_writer = csv.writer(file)

        # 각 행을 CSV 파일에 작성
        for row in rows:
            csv_writer.writerow(row)

def col3_to_CSV(list1,list2,list3, filename):
    # 3개의 리스트를 zip하여 행으로 묶음
    rows = zip(list1,list2,list3)

    # CSV 파일 열기
    with open(filename, 'w', newline='') as file:
        # CSV writer 생성
        csv_writer = csv.writer(file)

        # 각 행을 CSV 파일에 작성
        for row in rows:
            csv_writer.writerow(row)

def col4_to_CSV(list1,list2,list3,list4,filename):
    # 3개의 리스트를 zip하여 행으로 묶음
    rows = zip(list1,list2,list3,list4)

    # CSV 파일 열기
    with open(filename, 'w', newline='') as file:
        # CSV writer 생성
        csv_writer = csv.writer(file)

        # 각 행을 CSV 파일에 작성
        for row in rows:
            csv_writer.writerow(row)

def col15_to_CSV(list1,list2,list3,list4,list5,list6,list7,list8,list9,list10,list11,list12,list13,list14,list15,filename):
    # 3개의 리스트를 zip하여 행으로 묶음
    rows = zip(list1,list2,list3,list4,list5,list6,list7,list8,list9,list10,list11,list12,list13,list14,list15)

    
    # CSV 파일 열기
    with open(filename, 'w', newline='') as file:
        # CSV writer 생성
        csv_writer = csv.writer(file)

        # 각 행을 CSV 파일에 작성
        for row in rows:
            csv_writer.writerow(row)


# # 예시 리스트
# list1 = [1, 2, 3, 4, 5]
# list2 = ['a', 'b', 'c', 'd', 'e']
# list3 = [True, False, True, False, True]

# # CSV 파일로 내보내기
# ExportResult.col3_to_CSV(list1, list2, list3, '파일경로.csv')

# Lessrafim=['허윤진','사쿠라']
# ExportResult.col1_to_CSV(Lessrafim,"C:/Users/LeeKiYoon/Desktop/guidanceTEST/simulResult/0718/르세라핌.csv")