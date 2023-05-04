with open("font.txt") as f:
    text = f.read().split(",")
text2 = []
for i in range(len(text) // 2):
    text2.append(text[i * 2])
text3 = ",".join(text2)
with open("font2.txt", "w") as f:
    f.write(text3)
