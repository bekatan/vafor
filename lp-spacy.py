import spacy

nlp = spacy.load('en_core_web_sm')

doc = nlp('can you pass me the book and the smartphone') 
target = []

for token in doc:
    print(token.text, token.pos_, token.dep_)