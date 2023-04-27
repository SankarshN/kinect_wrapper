import face_recognition

# print("hello")

def get_encoding(image):
    pass
    return face_recognition.face_encodings(image)[0]

def find_person(image, original_encoding):
    loc = face_recognition.face_locations(image)
    enc = face_recognition.face_encodings(image, loc)
    if(len(enc) > 0):
        for (top, right, bottom, left), face_encoding in zip(loc, enc):
            center = ((top + bottom) // 2, (left + right) // 2)
            if face_recognition.compare_faces([original_encoding], face_encoding):
                return center
    return (-1, -1)