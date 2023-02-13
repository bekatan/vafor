import pandas as pd

models = ['text-davinci-003','text-curie-001', 'text-babbage-001', 'text-ada-001', ]
folders = ['_panda', '']
objects = pd.read_csv('text-davinci-003_panda_responses.csv', usecols=['target'])['target'].unique().tolist()

for folder in folders:
    for model in models:
        df = pd.read_csv(model+folder+'_responses.csv', usecols=['mark'])
        marks = df['mark'].to_list()
        print('Model:', model+folder)
        print('  Total accuracy: ', sum(marks), '%')
        print('  Accuracy on direct commands: ', sum(marks[::10])*10, '%')
        print('  Accuracy on each object:')
        for i, obj in enumerate(objects):
            print('   ', obj, ': ', sum(marks[i*10:(i+1)*10])*10, '%')
        print('\n------------------------\n')
