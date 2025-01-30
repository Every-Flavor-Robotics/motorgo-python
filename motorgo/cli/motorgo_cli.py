import click


@click.group()
def cli():
    pass


@cli.command()
def flash():
    print("Hello, World!")


if __name__ == "__main__":
    cli()
