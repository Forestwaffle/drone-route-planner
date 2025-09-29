print("범위를 정해 주시면 숫자를 맞춰 볼게요!!\n1.범위를 정해주세요\n2.예상보다 높으면 업 or u 입력 \n3.예상보다 낮으면 다운 or d 입력\n4.정답이면 정답 or a 입력 5. q입력은 종료 \n")
while True: #전체 반복 (정답 맞춘 후 종료 조건 r 키 외 입력하면 다시 반복)
    low = int(input('아래 범위 : ')) #낮은 숫자의 이상
    hi = int(input('위 범위 : ')) # 높은 숫자의 이하
    print("")

    while low > hi: # 낮은 숫자가 높은 숫자를 넘으면 이행
        print("범위를 다시 설정")
        low = int(input('아래 범위 : '))
        hi = int(input('위 범위 : '))

    while low <= hi: #이진탐색기법
        mid = (low + hi) // 2 
        print(f"예상 숫자: {mid}" )
        ans = input("업(u) / 다운(d) / 정답(a) : ")

        if ans == "a" or ans == "정답": 
            print("정답!")
            break
        elif ans == "u" or ans == "업":
            low = mid + 1
        elif ans == "d" or ans == "다운":
            hi = mid - 1
        elif ans == "q":
            print("종료")
            break
        else:
            print("업 / 다운 / 정답 중 하나만 입력")

    re = input("다시 하시겠습니까? ('r' 입력시 제시작, 그외 종료) : ") # 정답후 다시할지 결정
    print("")
    if re != "r":  #종료 조건
        print("게임 종료!")
        break 
