# import spacy
# from spacy import displacy

# nlp = spacy.load('en_core_web_sm')

# doc = nlp(u'Can you bring the remote to me?')

# displacy.serve(doc, style="dep")

# for token in doc:
#     print(token.text, token.pos_, token.dep_)

import os

import openai
openai.api_key = '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'
objects = ['banana', 'bottle', 'apple', 'chair', 'cup', 'keyboard', 'laptop', 'mouse', 'remote', 'scissors', 'speaker', 'cellphone', 'chair']
sentence = "I want to eat something."
prompt = "Decide which of these is the desired object in the prompt: {}\n".format(', '.join(objects))

prompt += """Prompt: "{}"
Desired object:""".format(sentence)

response = openai.Completion.create(
    model="text-davinci-003",
    prompt=prompt,
    temperature=0,
)
print(prompt)
print(response.choices[0].text)
for i in response.choices:
    print(i.text)
