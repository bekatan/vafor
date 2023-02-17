import pandas as pd

models = ['text-davinci-003','text-curie-001', 'text-babbage-001', 'text-ada-001', 
          'text-davinci-003_panda','text-curie-001_panda', 'text-babbage-001_panda', 'text-ada-001_panda', 
          'ada:ft-personal:ada-ft-0-3-2023-02-17-07-08-26', 'babbage:ft-personal-2023-02-17-07-51-46', 'curie:ft-personal-2023-02-17-08-29-04', 'davinci:ft-personal-2023-02-17-09-43-03']
objects = pd.read_csv('text-davinci-003_panda_responses.csv', usecols=['target'])['target'].unique().tolist()

for model in models:
    df = pd.read_csv(model+'_responses.csv', usecols=['mark'])
    marks = df['mark'].to_list()
    print('Model:', model)
    print('  Total accuracy: ', sum(marks), '%')
    print('  Accuracy on direct commands: ', sum(marks[::10])*10, '%')
    print('  Accuracy on indirect commands: ', (sum(marks) - sum(marks[::10]))*10//9, '%')
    print('  Accuracy on each object:')
    for i, obj in enumerate(objects):
        print('   ', obj, ': ', sum(marks[i*10:(i+1)*10])*10, '%')
    print('\n------------------------\n')
