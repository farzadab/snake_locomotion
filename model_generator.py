from jinja2 import Environment, PackageLoader, select_autoescape
import argparse


env = Environment(
    loader=PackageLoader('models', 'templates'),
    autoescape=select_autoescape(['html', 'xml'])
)
DEFAULT_OUTPUT_FILE = "snake.urdf"

def createNLinkSnake(n, outFile=DEFAULT_OUTPUT_FILE):
    template = env.get_template('snake.urdf.jinja2')
    links = range(n)
    with open(outFile, 'w') as out:
        model = template.render(model_name="snake", links=links, joints=zip(links[:-1], links[1:]))
        out.write(model)



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Simulate a multi-link pendulum')
    parser.add_argument('--nlinks', metavar='N', type=int, default=5, help='number of links')
    parser.add_argument('--out', metavar='OUT', type=str, default=DEFAULT_OUTPUT_FILE, help='output file name/path')
    args = parser.parse_args()

    createNLinkSnake(args.nlinks, args.out)
